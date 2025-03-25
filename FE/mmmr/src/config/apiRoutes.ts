// src/config/apiRoutes.ts

// 서버 주소 설정
const BASE_URL = 'http://70.12.246.168:8088';

// API 경로 설정
export const API_ROUTES = {
    // 프로필 관련 API
    profiles: {
        list: `${BASE_URL}/api/profiles`,
        availableCallsigns: `${BASE_URL}/api/profiles/available-callsigns`,
        add: `${BASE_URL}/api/profiles`,
        update: (profileId: string) => `${BASE_URL}/api/profiles/${profileId}`,
        delete: (profileId: string) => `${BASE_URL}/api/profiles/${profileId}`,
    },

    // 계정 관리 관련 API
    accounts: {
        checkEmailExists: `${BASE_URL}/api/accounts/email-exists`,
        signUp: `${BASE_URL}/api/accounts`,
        changePassword: `${BASE_URL}/api/accounts/password`,
    },

    // 로그인 / 인증 관련 API
    auth: {
        validate: `${BASE_URL}/api/auth/validate`,
        refresh: `${BASE_URL}/api/auth/refresh`,
        logout: `${BASE_URL}/api/auth/logout`,
        login: `${BASE_URL}/api/auth/login`,
    },

    // 버스 정보 DB 관련 API
    busInformation: {
        uploadExcel: `${BASE_URL}/api/bus-information`,
    },
};
