// src/config/apiRoutes.ts

// 서버 주소 설정
const BASE_URL = "http://70.12.246.168:8088";

// API 경로 설정
export const API_ROUTES = {
    // 프로필 관련 API
    profiles: {
        list: `${BASE_URL}/api/profiles`,
        callsigns: `${BASE_URL}/api/profiles/callsigns`,
        add: `${BASE_URL}/api/profiles`,
        update: (profileId: number) => `${BASE_URL}/api/profiles/${profileId}`,
        delete: (profileId: number) => `${BASE_URL}/api/profiles/${profileId}`,
    },

    // 계정 관리 관련 API
    accounts: {
        emailExists: `${BASE_URL}/api/accounts/email-exists`,
        codeVerification: (email: string) => `${BASE_URL}/api/mail/verification/${email}`,
        sendCodes: `${BASE_URL}/api/mail/codes`,
        signUp: `${BASE_URL}/api/accounts`,
        changePassword: `${BASE_URL}/api/accounts/password`,
        sendPassword: `${BASE_URL}/api/mail/password`,
    },

    // 로그인 / 인증 관련 API
    auth: {
        validate: `${BASE_URL}/api/auth/validate`,
        refresh: `${BASE_URL}/api/auth/refresh`,
        logout: `${BASE_URL}/api/auth/logout`,
        login: `${BASE_URL}/api/auth/login`,
    },

    // Todo 리스트 관련 API
    todos: {
        listByProfile: (profileId: number) => `${BASE_URL}/api/todos/profile/${profileId}`,
        listByStatus: (profileId: number, status: string) =>
            `${BASE_URL}/api/todos/profile/${profileId}/status?status=${status}`,
        detail: (todoId: number) => `${BASE_URL}/api/todos/${todoId}`,
        add: `${BASE_URL}/api/todos`,
        update: (todoId: number) => `${BASE_URL}/api/todos/${todoId}`,
        delete: (todoId: number) => `${BASE_URL}/api/todos/${todoId}`,
        toggle: (todoId: number) => `${BASE_URL}/api/todos/${todoId}/toggle`,
    },

    // Schedule 리스트 관련 API
    schedule: {
        listByProfile: `${BASE_URL}/api/schedules/profile`,
        detail: (id: number) => `${BASE_URL}/api/schedules/${id}`,
        add: `${BASE_URL}/api/schedules`,
        update: (id: number) => `${BASE_URL}/api/schedules/${id}`,
        delete: (id: number) => `${BASE_URL}/api/schedules/${id}`,
    },

    youtube: `${BASE_URL}/api/video`,
    news: `${BASE_URL}/api/news`,
};

export default API_ROUTES;
