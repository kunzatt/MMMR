import requests
import json
from dotenv import load_dotenv
import os
from localServer_gcp import logger
import openai
from datetime import datetime, timedelta

load_dotenv()

server_url = os.getenv("SERVER_URL")
email = os.getenv("EMAIL")
password = os.getenv("PASSWORD") 
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
openai.api_key = OPENAI_API_KEY

profile_id_map = {}
nickname_map = {}


def login():
    login_data = {
        "email": email,
        "password": password
    }
    try:
        response = requests.post(server_url+"auth/login", data=json.dumps(login_data), headers={"Content-Type": "application/json"})
        if response.status_code == 200:
            data = response.json()
            logger.info(f"로그인 성공: {data}")
            return {
                "access_token": data['data']['accessToken'],
                "refresh_token": data['data']['refreshToken']
            }
        else:
            logger.error(f"로그인 실패: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        logger.error(f"로그인 요청 중 오류: {e}")
        return None

def refresh_access_token(refresh_token):
    try:
        refresh_data = {
            "token": refresh_token
        }
        response = requests.post(
            server_url+"auth/refresh", 
            data=json.dumps(refresh_data), 
            headers={"Content-Type": "application/json"}
        )
        
        if response.status_code == 200:
            data = response.json()
            logger.info("액세스 토큰 갱신 성공")
            return {
                "access_token": data['data']['accessToken'],
                "refresh_token": data['data']['refreshToken']  # 새 리프레시 토큰이 발급될 수 있음
            }
        else:
            logger.error(f"액세스 토큰 갱신 실패: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        logger.error(f"토큰 갱신 요청 중 오류: {e}")
        return None

def make_authenticated_request(url, method="GET", headers=None, data=None, params=None, access_token=None, refresh_token=None):
    if headers is None:
        headers = {}
    
    if access_token:
        headers["Authorization"] = f"Bearer {access_token}"
    
    try:
        if method.upper() == "GET":
            response = requests.get(url, headers=headers, params=params)
        elif method.upper() == "POST":
            response = requests.post(url, headers=headers, json=data if data else None, params=params)
        elif method.upper() == "PUT":
            response = requests.put(url, headers=headers, json=data if data else None, params=params)
        else:
            logger.error(f"지원하지 않는 HTTP 메서드: {method}")
            return None
        
        # 액세스 토큰 만료 확인 (401 Unauthorized)
        if response.status_code == 401 and refresh_token:
            logger.info("액세스 토큰이 만료되었습니다. 갱신을 시도합니다.")
            new_tokens = refresh_access_token(refresh_token)
            
            if new_tokens:
                # 새 액세스 토큰으로 헤더 업데이트
                headers["Authorization"] = f"Bearer {new_tokens['access_token']}"
                
                # 요청 재시도
                if method.upper() == "GET":
                    response = requests.get(url, headers=headers, params=params)
                elif method.upper() == "POST":
                    response = requests.post(url, headers=headers, json=data if data else None, params=params)
                elif method.upper() == "PUT":
                    response = requests.put(url, headers=headers, json=data if data else None, params=params)

                
                # 성공적인 응답이면 새 토큰과 함께 반환
                if response.status_code == 200:
                    return {
                        "response": response,
                        "new_tokens": new_tokens
                    }
                else:
                    logger.error(f"토큰 갱신 후에도 요청 실패: {response.status_code}")
                    return None
            else:
                logger.error("토큰 갱신 실패")
                return None
        
        # 일반적인 성공 응답
        if response.status_code == 200:
            return {
                "response": response,
                "new_tokens": None  # 토큰 갱신이 없었음
            }
        else:
            logger.error(f"API 요청 실패: {response.status_code}")
            return None
            
    except requests.exceptions.RequestException as e:
        logger.error(f"API 요청 중 오류: {e}")
        return None
    
def getProfileId(callSign, access_token, refresh_token=None):
    global profile_id_map
    if callSign in profile_id_map:
        logger.info(f"캐시된 프로필 ID 사용: {profile_id_map[callSign]}")
        return profile_id_map[callSign], None
    try:
        result = make_authenticated_request(
            server_url+"profiles/callsigns/" + callSign, 
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return None, None
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        
        data = response.json()
        logger.info(f"프로필 데이터: {data}")
        
        profile_id_map[callSign] = data["data"]
        logger.info(f"프로필 ID 캐시: {profile_id_map[callSign]}")
        if new_tokens:
            return data["data"], new_tokens
        else:
            return data["data"], None
            
    except Exception as e:
        logger.error(f"프로필 처리 중 오류: {e}")
        return None, None

def getNickname(profileId, access_token, refresh_token=None):
    global nickname_map
    if profileId in nickname_map:
        logger.info(f"캐시된 닉네임 사용: {nickname_map[profileId]}")
        return nickname_map[profileId], None
    try:
        result = make_authenticated_request(
            server_url+"profiles", 
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return None, None

        response = result["response"]
        new_tokens = result["new_tokens"]

        profiles = response.json()
        logger.info(f"프로필 데이터: {profiles}")

        nickname = None
        for profile in profiles["data"]:
            if profile["id"] == profileId:
                nickname = profile["nickname"]
                break
        logger.info(f"사용자 닉네임: {nickname}")
        nickname_map[profileId] = nickname
        logger.info(f"닉네임 캐시: {nickname_map}")

        if new_tokens:
            return nickname, new_tokens
        else:
            return nickname, None
        
    except Exception as e:
        logger.error(f"프로필 처리 중 오류: {e}")
        return None, None

def getWeather(access_token, refresh_token=None):
    try:
        result = make_authenticated_request(
            server_url+"weather", 
            headers={"Content-Type": "application/json"},
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return None, None
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        
        data = response.json()
        ret = data["data"]["clothingAdvice"] + "\n" + data["data"]["umbrellaAdvice"]

        logger.info(f"날씨 데이터: {data}")
        
        if new_tokens:
            return ret, new_tokens
        else:
            return ret, None
        
    except Exception as e:
        logger.error(f"날씨 처리 중 오류: {e}")
        return None, None

def getNews(id, access_token, refresh_token=None):
    try:
        if id > 5 or id <= 0:
            logger.error(f"지원하지 않는 뉴스 ID: {id}")
            return None, None
        result = make_authenticated_request(
            server_url+"news", 
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return None, None
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        
        if id == 0:
            # 토큰이 갱신되었으면 새 토큰 반환, 아니면 None 반환
            if new_tokens:
                return "success", new_tokens
            else:
                return "success", None
        
        data = response.json()
        news_item = None
        logger.info(f"뉴스 데이터: {data}")
        for item in data:
            if item["id"] == id:
                news_item = item
                break
        
        if not news_item:
            logger.error(f"ID {id}에 해당하는 뉴스를 찾을 수 없음")
            return None, new_tokens
        
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            logger.error("OpenAI API 키가 설정되지 않음")
            return None, new_tokens

        openai_url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {openai_api_key}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": "gpt-3.5-turbo",
            "messages": [
                {"role": "system", "content": "뉴스 내용을 2줄로 간결하게 요약해주세요."},
                {"role": "user", "content": news_item["content"]}
            ],
            "temperature": 0.7,
            "max_tokens": 100
        }
        
        openai_response = requests.post(openai_url, headers=headers, json=payload)
        if openai_response.status_code == 200:
            summary = openai_response.json()["choices"][0]["message"]["content"].strip()
            return summary, new_tokens
        else:
            logger.error(f"OpenAI API 요청 실패: {openai_response.status_code}, {openai_response.text}")
            return None, new_tokens
    except Exception as e:
        logger.error(f"뉴스 처리 중 오류: {e}")
        return None, None
    
def getSchedules(profileId, access_token, refresh_token=None, day="today"):
    try:
        params = {
            "profileId": profileId
        }
        result = make_authenticated_request(
            server_url+"schedules/profile", 
            params=params,
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        
        if not result:
            return None, None
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        if new_tokens:
            access_token = new_tokens["access_token"]
            refresh_token = new_tokens["refresh_token"]

        nickname = getNickname(profileId, access_token, refresh_token)[0]
        data = response.json()
        logger.info(f"일정 데이터: {data}")        
        
        today = datetime.now()
        if day == "" :
            day = "today"
        if day == "today":
            target_date_start = today.strftime("%Y-%m-%d")
            target_date_end = target_date_start
        elif day == "tomorrow":
            tomorrow = today + timedelta(days=1)
            target_date_start = tomorrow.strftime("%Y-%m-%d")
            target_date_end = target_date_start
        elif day == "this_week":
            days_until_sunday = 6 - today.weekday() if today.weekday() <= 6 else 0
            sunday = today + timedelta(days=days_until_sunday)
            target_date_start = today.strftime("%Y-%m-%d")
            target_date_end = sunday.strftime("%Y-%m-%d")
        elif day == "next_week":
            days_until_next_monday = 7 - today.weekday() if today.weekday() < 7 else 1
            next_monday = today + timedelta(days=days_until_next_monday)
            next_sunday = next_monday + timedelta(days=6)
            target_date_start = next_monday.strftime("%Y-%m-%d")
            target_date_end = next_sunday.strftime("%Y-%m-%d")
        else:
            logger.error(f"지원하지 않는 day 값: {day}")
            return None, new_tokens
        
        target_date_start_obj = datetime.strptime(target_date_start, "%Y-%m-%d")
        target_date_end_obj = datetime.strptime(target_date_end, "%Y-%m-%d")
        
        logger.info(f"필터링 기간: {target_date_start} ~ {target_date_end}")
        
        filtered_schedules = []
        for schedule in data["data"]:
            start_date = datetime.strptime(schedule["startDate"], "%Y-%m-%d %H:%M:%S").strftime("%Y-%m-%d")
            end_date = datetime.strptime(schedule["endDate"], "%Y-%m-%d %H:%M:%S").strftime("%Y-%m-%d")
            
            start_date_obj = datetime.strptime(start_date, "%Y-%m-%d")
            end_date_obj = datetime.strptime(end_date, "%Y-%m-%d")
            
            if (start_date_obj <= target_date_end_obj and end_date_obj >= target_date_start_obj):
                filtered_schedules.append(schedule)
        
        if not filtered_schedules:
            logger.info(f"{day}에 해당하는 일정이 없습니다.")
            return f"{nickname}님 일정이 없습니다.", new_tokens
        
        # 필터링된 일정을 요약
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            logger.error("OpenAI API 키가 설정되지 않음")
            return filtered_schedules, new_tokens  # 요약 없이 원본 일정만 반환

        openai_url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {openai_api_key}",
            "Content-Type": "application/json"
        }
        
        # 모든 일정을 하나의 문자열로 결합
        schedules_text = "\n".join([f"{schedule['title']} ({schedule['startDate']} ~ {schedule['endDate']})" for schedule in filtered_schedules])
        
        payload = {
            "model": "gpt-3.5-turbo",
            "messages": [
                {"role": "system", "content": f"{nickname}님의 {day}의 일정을 2-3줄로 간결하게 요약해주세요."},
                {"role": "user", "content": schedules_text}
            ],
            "temperature": 0.7,
            "max_tokens": 100
        }
        
        openai_response = requests.post(openai_url, headers=headers, json=payload)
        if openai_response.status_code == 200:
            summary = openai_response.json()["choices"][0]["message"]["content"].strip()
            logger.info(f"{day} 일정 요약: {summary}")
            return summary, new_tokens
        else:
            logger.error(f"OpenAI API 요청 실패: {openai_response.status_code}, {openai_response.text}")
            return filtered_schedules, new_tokens  # 요약 실패 시 원본 일정만 반환
        
    except Exception as e:
        logger.error(f"일정 처리 중 오류: {e}")
        return None, None
    
def getDevices(access_token, refresh_token=None):
    try:
        result = make_authenticated_request(
            server_url+"devices", 
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return None, None
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        
        data = response.json()
        logger.info(f"장치 데이터: {data}")
        
        if new_tokens:
            return data, new_tokens
        else:
            return data, None
        
    except Exception as e:
        logger.error(f"장치 처리 중 오류: {e}")
        return None, None

def deviceUpdate(deviceId, turned, access_token, refresh_token=None):
    try:

        params = {
            "deviceId": deviceId,
            "turned": turned
        }
        result = make_authenticated_request(
            server_url+"devices/" + str(deviceId) + "/update", 
            "PUT",
            headers={"Content-Type": "application/json"},
            params=params,
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return False
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        
        data = response.json()
        logger.info(f"장치 업데이트 데이터: {data}")
        
        if new_tokens:
            return True, new_tokens
        else:
            return True, None
        
    except Exception as e:
        logger.error(f"장치 업데이트 중 오류: {e}")
        return False, None

def getGreeting(profileId, access_token, refresh_token=None):
    try:
        logger.info(f"프로필 ID: {profileId}")
        nickname, new_tokens = getNickname(profileId, access_token, refresh_token)
        if new_tokens:
            access_token = new_tokens["access_token"]
            refresh_token = new_tokens["refresh_token"]

        
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            logger.error("OpenAI API 키가 설정되지 않음")
            return f"안녕하세요, {nickname}님!", new_tokens 
        openai_url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {openai_api_key}",
            "Content-Type": "application/json"
        }
        
        
        payload = {
            "model": "gpt-3.5-turbo",
            "messages": [
                {"role": "system", "content": "당신은 IoT 제어, 일정, 교통, 뉴스, 날씨 등의 정보를 제공하는 음성 비서입니다. 사용자에게 친절하게 짧은 인사말을 만들어주세요. 최대 3문장 이내로 작성해주세요."},
                {"role": "user", "content": f"{nickname}에게 전할 인사말을 만들어주세요. (예 : '안녕하세요, {nickname}님! 오늘은 어떤 일정이 있나요?', '안녕하세요, {nickname}님! 좋은 하루입니다!', '안녕하세요, {nickname}님! 무엇이든 도와드리겠습니다.'),"}
            ],
            "temperature": 0.7,
            "max_tokens": 100
        }
        
        openai_response = requests.post(openai_url, headers=headers, json=payload)
        if openai_response.status_code == 200:
            summary = openai_response.json()["choices"][0]["message"]["content"].strip()
            logger.info(f"생성된 인사말말: {summary}")
            return summary, new_tokens
        else:
            logger.error(f"OpenAI API 요청 실패: {openai_response.status_code}, {openai_response.text}")
            return f"안녕하세요, {nickname}님!", new_tokens  # 요약 실패 시 원본 일정만 반환
    except Exception as e:
        logger.error(f"인사말 생성 중 오류: {e}")
        return None, None
    
def getTime():
    now = datetime.now()
    hour = now.hour
    minute = now.minute
    result = f"현재 시각은 {hour}시 {minute}분입니다."
    return result

def getTransportation(profileId, access_token, refresh_token=None, type="BUS"):
    try:
        result = make_authenticated_request(
            server_url+F"transportations/profile/{str(profileId)}/arrivals", 
            
            access_token=access_token, 
            refresh_token=refresh_token
        )
        response = result["response"].json()
        new_tokens = result["new_tokens"]

        logger.info(f"result : {response}")
        summary_data = ""
        if type == "BUS":
            bus_data = [item for item in response if item["type"] == "BUS"]            
            if bus_data:
                for i, bus in enumerate(bus_data):
                    if i > 0:
                        summary_data += ", "
                    summary_data += f"{bus['station']}에 {bus['direction']} {bus['number']}번 버스는 {bus['information']}"                
            else:
                summary_data="현재 버스 정보가 없습니다."
        else:
            metro_data = [item for item in response if item["type"] == "METRO"]
            if metro_data:
                for i, metro in enumerate(metro_data):
                    if i > 0:
                        summary_data += ", "
                    summary_data += f"{metro['station']} {metro['number']} {metro['direction']} 열차는 {metro['information']}"
            else:
                summary_data="현재 지하철 정보가 없습니다."
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            logger.error("OpenAI API 키가 설정되지 않음")
            if type == "bus":
                return "버스 정보를 불러오지 못하였습니다.", None
            else:
                return "지하철 정보를 불러오지 못하였습니다.", None
            
        openai_url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {openai_api_key}",
            "Content-Type": "application/json"
        }
        
        
        payload = {
            "model": "gpt-3.5-turbo",
            "messages": [
                 {"role": "system", "content": "당신은 교통 정보를 간결하게 요약해주는 비서입니다. 버스 또는 지하철 정보를 2-3줄 내로 간결하게 요약해주세요."},
                 {"role": "user", "content": f"다음 정보를 2-3줄 이내로 간결하게 요약해주세요: {summary_data}"}
            ],
            "temperature": 0.7,
            "max_tokens": 100
        }
        
        openai_response = requests.post(openai_url, headers=headers, json=payload)
        if openai_response.status_code == 200:
            summary = openai_response.json()["choices"][0]["message"]["content"].strip()
            logger.info(f"생성된 교통 정보: {summary}")
            return summary, new_tokens
        else:
            logger.error(f"OpenAI API 요청 실패: {openai_response.status_code}, {openai_response.text}")
            return "교통 정보를 불러오지 못하였습니다.", new_tokens 
        
    except Exception as e:
        logger.error(f"교통 정보 생성 중 오류 : {e}")
        return "교통 정보를 불러오지 못하였습니다.", None


        


if __name__ == "__main__":
    # 초기 로그인으로 토큰 획득
    tokens = login()
    if not tokens:
        print("로그인 실패")
        exit(1)
        
    access_token = tokens["access_token"]
    refresh_token = tokens["refresh_token"]
    
    # 프로필 가져오기 (토큰 갱신 포함)
    #profiles, new_tokens = getProfileId(access_token, refresh_token)
    
    # 토큰이 갱신되었으면 업데이트
    # if new_tokens:
    #     access_token = new_tokens["access_token"]
    #     refresh_token = new_tokens["refresh_token"]
    #     logger.info("토큰이 갱신되었습니다.")

    result, new_tokens = getTransportation("미미", access_token, refresh_token, "bus")
    
    print(result)
    # if profiles:
    #     print("프로필 데이터:")
    #     print(profiles)
    #     for profile in profiles:
    #         print(f"이름: {profile['name']}, 이메일: {profile['email']}")
        
    #     # 뉴스 가져오기 (토큰 갱신 포함)
    #     news, new_tokens = getNews(access_token, refresh_token, 1)
        
    #     # 토큰이 갱신되었으면 업데이트
    #     if new_tokens:
    #         access_token = new_tokens["access_token"]
    #         refresh_token = new_tokens["refresh_token"]
    #         logger.info("토큰이 갱신되었습니다.")
            
    #     if news:
    #         print(f"뉴스 요약: {news}")
    # else:
    #     print("프로필 정보를 가져오는 데 실패했습니다.")