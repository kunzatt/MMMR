# 📚 미미미러(MMMR)

## 📋 목차

1. [프로젝트 소개](#-프로젝트-소개)
2. [팀원 소개](#-팀원-소개)
3. [시스템 아키텍처](#-시스템-아키텍처)
4. [기술 스택](#-기술-스택)
5. [주요 기능](#-주요-기능)
6. [설치 및 실행 방법](#-설치-및-실행-방법)
7. [기여하기](#-기여하기)

## 🎯 프로젝트 소개

### 개요

MMMR는 일상생활에 유용한 정보를 제공하는 스마트미러입니다. 시간, 날씨, 일정 뿐만 아니라 인공지능 비서, 스마트홈 제어, 자율주행 홈캠 모니터링 등 다양한 기능을 갖춘 올인원 스마트 디바이스입니다.

### 영상
[![YouTube Video](이미지예시)](https://www.youtube.com/)

### 핵심 기능

| 기능              | 설명 |
|-----------------|----|
| 📱 인공지능 비서      | 음성 명령을 통한 다양한 정보 검색 및 시스템 제어 |
| 🎨 맞춤형 UI       | 사용자 선호도에 맞춘 인터페이스 커스터마이징 |
| 📸 자율주행 홈캠 스트리밍 | 실시간 홈 모니터링 시스템 |
| 💡 스마트홈 제어      | 조명, 가전제품 등 IoT 기기 통합 제어 |
| 🔔 알림 시스템       | 캘린더, 메시지, 일정 관련 실시간 알림 |

## 👥 팀원 소개

| 이름  | 담당                     |
|-----|------------------------|
| 엄도윤 | 팀장, AI                 |
| 박태하 | 스마트 홈 시스템              |
| 정영진 | 자율주행 및 홈캠 스트리밍         |
| 신유정 | Frontend               |
| 김용명 | Backend                |
| 권희주 | Infra (CI/CD), Backend |

## 🔍 시스템 아키텍처

<img src="./assets/시스템 아키텍쳐.png" width="800" height="600" />

### ERD
<img width="800" height="600" src="./assets/ERD 다이어그램.png" />

## 🛠 기술 스택

### 🤖 AI & 스마트홈 시스템
[![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=Python&logoColor=white)](/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=flat-square&logo=Raspberry%20Pi&logoColor=white)](/)
[![ESP32](https://img.shields.io/badge/ESP32-E7352C?style=flat-square&logo=Espressif&logoColor=white)](/)
[![Google Cloud Platform](https://img.shields.io/badge/Google%20Cloud-4285F4?style=flat-square&logo=Google%20Cloud&logoColor=white)](/)
[![Speech-to-Text](https://img.shields.io/badge/Speech%20to%20Text-4285F4?style=flat-square&logo=Google%20Cloud&logoColor=white)](/)
[![Text-to-Speech](https://img.shields.io/badge/Text%20to%20Speech-4285F4?style=flat-square&logo=Google%20Cloud&logoColor=white)](/)
[![Porcupine](https://img.shields.io/badge/Porcupine-885630?style=flat-square&logo=Piaggio&logoColor=white)](/)
[![FastAPI](https://img.shields.io/badge/FastAPI-009688?style=flat-square&logo=FastAPI&logoColor=white)](/)
[![WebSocket](https://img.shields.io/badge/WebSocket-010101?style=flat-square&logo=Socket.io&logoColor=white)](/)
[![OpenAI API](https://img.shields.io/badge/OpenAI%20API-412991?style=flat-square&logo=OpenAI&logoColor=white)](/)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat-square&logo=Arduino&logoColor=white)](/)
[![Qt](https://img.shields.io/badge/Qt-41CD52?style=flat-square&logo=Qt&logoColor=white)](/)

### 🚗 자율주행 & 홈캠 시스템
[![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=Python&logoColor=white)](/)
[![ROS2](https://img.shields.io/badge/ROS2-22314E?style=flat-square&logo=ROS&logoColor=white)](/)
[![MORAI](https://img.shields.io/badge/MORAI-00599C?style=flat-square&logo=Simulator&logoColor=white)](/)
[![Flask](https://img.shields.io/badge/Flask-000000?style=flat-square&logo=Flask&logoColor=white)](/)
[![Cloudflare](https://img.shields.io/badge/Cloudflare-F38020?style=flat-square&logo=Cloudflare&logoColor=white)](/)
[![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=OpenCV&logoColor=white)](/)
[![WebSocket](https://img.shields.io/badge/WebSocket-010101?style=flat-square&logo=Socket.io&logoColor=white)](/)

### 🎨 Frontend
[![Next.js](https://img.shields.io/badge/Next.js-000000?style=flat-square&logo=Next.js&logoColor=white)](/)
[![TypeScript](https://img.shields.io/badge/TypeScript-3178C6?style=flat-square&logo=TypeScript&logoColor=white)](/)
[![Tailwind CSS](https://img.shields.io/badge/Tailwind%20CSS-38B2AC?style=flat-square&logo=Tailwind%20CSS&logoColor=white)](/)
[![PWA](https://img.shields.io/badge/PWA-5A0FC8?style=flat-square&logo=PWA&logoColor=white)](/)

### 💻 Backend
[![Java](https://img.shields.io/badge/Java-007396?style=flat-square&logo=Java&logoColor=white)](/)
[![Spring Boot](https://img.shields.io/badge/Spring%20Boot-6DB33F?style=flat-square&logo=Spring%20Boot&logoColor=white)](/)
[![JPA](https://img.shields.io/badge/JPA-6DB33F?style=flat-square&logo=Hibernate&logoColor=white)](/)
[![MySQL](https://img.shields.io/badge/MySQL-4479A1?style=flat-square&logo=MySQL&logoColor=white)](/)
[![Redis](https://img.shields.io/badge/Redis-DC382D?style=flat-square&logo=Redis&logoColor=white)](/)
[![JWT](https://img.shields.io/badge/JWT-000000?style=flat-square&logo=JSON%20Web%20Tokens&logoColor=white)](/)
[![Spring Security](https://img.shields.io/badge/Spring%20Security-6DB33F?style=flat-square&logo=Spring%20Security&logoColor=white)](/)

### ⚙️ DevOps
[![Git](https://img.shields.io/badge/Git-F05032?style=flat-square&logo=Git&logoColor=white)](/)
[![GitLab](https://img.shields.io/badge/GitLab-FC6D26?style=flat-square&logo=GitLab&logoColor=white)](/)
[![Jira](https://img.shields.io/badge/Jira-0052CC?style=flat-square&logo=Jira&logoColor=white)](/)
[![NGINX](https://img.shields.io/badge/NGINX-009639?style=flat-square&logo=NGINX&logoColor=white)](/)
[![Docker](https://img.shields.io/badge/Docker-2496ED?style=flat-square&logo=Docker&logoColor=white)](/)
[![Jenkins](https://img.shields.io/badge/Jenkins-D24939?style=flat-square&logo=Jenkins&logoColor=white)](/)
[![AWS EC2](https://img.shields.io/badge/AWS%20EC2-FF9900?style=flat-square&logo=Amazon%20EC2&logoColor=white)](/)
[![Swagger](https://img.shields.io/badge/Swagger-85EA2D?style=flat-square&logo=Swagger&logoColor=black)](/)

## 💫 주요 기능
blah blah

## 📋 설치 및 실행 방법

### 하드웨어 요구사항
- Raspberry Pi 4 (8GB RAM 권장)
- 거울 필름
- LCD 디스플레이
- 마이크 어레이 (음성 인식용)
- ESP32 (IoT 기기 제어용)

### 소프트웨어 설치
```bash
# 저장소 복제
git clone https://github.com/yourusername/MMMR.git
cd MMMR

# 백엔드 서비스 실행 (Docker)
docker-compose up -d

# 프론트엔드 설치 및 실행
cd frontend
npm install
npm run build
npm start

# AI 서비스 설치 및 실행
cd ../ai_service
pip install -r requirements.txt
python main.py
```

### 환경 설정
Backend 폴더 내에 env 파일을 통해 다음 항목을 구성할 수 있습니다:
- API 키 설정 (날씨, 대중교통, 유튜브 API 등)
- JWT 설정
- 기타 설정 등

### 데이터베이스 설치
MySQL에서 /assets에 있는 SQL 스크립트를 실행합니다.

## 🤝 기여하기

프로젝트에 기여하고 싶으시다면:
1. 저장소를 포크합니다.
2. 기능 브랜치를 생성합니다: `git checkout -b feature/amazing-feature`
3. 변경사항을 커밋합니다: `git commit -m 'Add some amazing feature'`
4. 포크한 저장소에 푸시합니다: `git push origin feature/amazing-feature`
5. Pull Request를 생성합니다.

---

© 2025 MMMR. All Rights Reserved.