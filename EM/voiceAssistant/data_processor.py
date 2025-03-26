import requests
import json
from dotenv import load_dotenv
import os
from localServer import logger
import openai

Login_url = os.getenv("LOGIN_URL")
GetNews_url = os.getenv("GETNEWS_URL")
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
        response = requests.post(Login_url, data=json.dumps(login_data), headers={"Content-Type": "application/json"})
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
    

def get_news(access_token, id=0):
    print("a")
    try:
        response = requests.get(GetNews_url, headers={"Authorization": f"Bearer {access_token}"})
        if response.status_code == 200:
            if id == 0:
                return "success"
            data = response.json()
            news_item = None
            logger.info(f"뉴스 데이터: {data}")
            for item in data:
                if item["id"] == id:
                    news_item = item
                    break
            
            if not news_item:
                logger.error(f"ID {id}에 해당하는 뉴스를 찾을 수 없음")
                return None
            
            openai_api_key = os.getenv("OPENAI_API_KEY")
            if not openai_api_key:
                logger.error("OpenAI API 키가 설정되지 않음")
                return None

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
                logger.info(f"OpenAI 요청 성공: {summary}")
                return summary
            else:
                logger.error(f"OpenAI API 요청 실패: {openai_response.status_code}, {openai_response.text}")
                return None
        else:
            logger.error(f"뉴스 가져오기 실패: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        logger.error(f"뉴스 요청 중 오류: {e}")
        return None