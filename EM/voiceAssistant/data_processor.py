import requests
import json
from dotenv import load_dotenv
import os
from localServer_gcp import logger
import openai

load_dotenv()

login_url = os.getenv("LOGIN_URL")
refresh_token_url = os.getenv("REFRESH_TOKEN_URL")  # .env 파일에 리프레시 토큰 URL 추가 필요
getNews_url = os.getenv("GETNEWS_URL")
getProfiles_url = os.getenv("GETPROFILES_URL")
email = os.getenv("EMAIL")
password = os.getenv("PASSWORD") 
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
openai.api_key = OPENAI_API_KEY


def login():
    login_data = {
        "email": email,
        "password": password
    }
    try:
        response = requests.post(login_url, data=json.dumps(login_data), headers={"Content-Type": "application/json"})
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
            "refreshToken": refresh_token
        }
        response = requests.post(
            refresh_token_url, 
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


def make_authenticated_request(url, method="GET", headers=None, data=None, access_token=None, refresh_token=None):
    if headers is None:
        headers = {}
    
    if access_token:
        headers["Authorization"] = f"Bearer {access_token}"
    
    try:
        if method.upper() == "GET":
            response = requests.get(url, headers=headers)
        elif method.upper() == "POST":
            response = requests.post(url, headers=headers, data=json.dumps(data) if data else None)
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
                    response = requests.get(url, headers=headers)
                elif method.upper() == "POST":
                    response = requests.post(url, headers=headers, data=json.dumps(data) if data else None)
                
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


def getProfiles(access_token, refresh_token=None):
    try:
        result = make_authenticated_request(
            getProfiles_url, 
            access_token=access_token, 
            refresh_token=refresh_token
        )
        
        if not result:
            return None, None
            
        response = result["response"]
        new_tokens = result["new_tokens"]
        
        data = response.json()
        logger.info(f"프로필 데이터: {data}")
        
        # 토큰이 갱신되었으면 새 토큰 반환, 아니면 None 반환
        if new_tokens:
            return data, new_tokens
        else:
            return data, None
            
    except Exception as e:
        logger.error(f"프로필 처리 중 오류: {e}")
        return None, None


def getNews(access_token, refresh_token=None, id=0):
    try:
        result = make_authenticated_request(
            getNews_url, 
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
    
    
if __name__ == "__main__":
    # 초기 로그인으로 토큰 획득
    tokens = login()
    if not tokens:
        print("로그인 실패")
        exit(1)
        
    access_token = tokens["access_token"]
    refresh_token = tokens["refresh_token"]
    
    # 프로필 가져오기 (토큰 갱신 포함)
    profiles, new_tokens = getProfiles(access_token, refresh_token)
    
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