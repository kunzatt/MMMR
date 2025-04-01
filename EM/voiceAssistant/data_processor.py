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
address = os.getenv("ADDRESS")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
openai.api_key = OPENAI_API_KEY


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
        
        if new_tokens:
            return data, new_tokens
        else:
            return data, None
            
    except Exception as e:
        logger.error(f"프로필 처리 중 오류: {e}")
        return None, None

def getWeather(access_token, refresh_token=None):
    try:
        result = make_authenticated_request(
            server_url+"weather", 
            "POST",
            headers={"Content-Type": "application/json"},
            data=address,
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

def getNews(access_token, refresh_token=None, id=0):
    try:
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
    
def getSchedules(callsign, access_token, refresh_token=None, day="today"):
    try:
        logger.info(f"일정 요청: {callsign}, {day}")
        profileId_data, new_tokens = getProfileId(callsign, access_token, refresh_token)
        profileId = profileId_data["data"]
        logger.info(f"프로필 ID: {profileId}")
        if new_tokens:
            access_token = new_tokens["access_token"]
            refresh_token = new_tokens["refresh_token"]

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
        if not new_tokens:
            new_tokens = result["new_tokens"]
        
        data = response.json()
        logger.info(f"일정 데이터: {data}")
        
        
        
        today = datetime.now()
        
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
            return "일정이 없습니다.", new_tokens
        
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
                {"role": "system", "content": f"{day}의 일정을 2-3줄로 간결하게 요약해주세요."},
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
    
    
if __name__ == "__main__":
    # 초기 로그인으로 토큰 획득
    tokens = login()
    if not tokens:
        print("로그인 실패")
        exit(1)
        
    access_token = tokens["access_token"]
    refresh_token = tokens["refresh_token"]
    
    # 프로필 가져오기 (토큰 갱신 포함)
    profiles, new_tokens = getProfileId(access_token, refresh_token)
    
    # 토큰이 갱신되었으면 업데이트
    if new_tokens:
        access_token = new_tokens["access_token"]
        refresh_token = new_tokens["refresh_token"]
        logger.info("토큰이 갱신되었습니다.")
    
    if profiles:
        print("프로필 데이터:")
        print(profiles)
        for profile in profiles:
            print(f"이름: {profile['name']}, 이메일: {profile['email']}")
        
        # 뉴스 가져오기 (토큰 갱신 포함)
        news, new_tokens = getNews(access_token, refresh_token, 1)
        
        # 토큰이 갱신되었으면 업데이트
        if new_tokens:
            access_token = new_tokens["access_token"]
            refresh_token = new_tokens["refresh_token"]
            logger.info("토큰이 갱신되었습니다.")
            
        if news:
            print(f"뉴스 요약: {news}")
    else:
        print("프로필 정보를 가져오는 데 실패했습니다.")