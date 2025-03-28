import { API_ROUTES } from "./apiRoutes"; // 기존 apiRoutes 파일을 import
import { useRouter } from "next/navigation"; // next/navigation에서 router 가져오기

export const getTokens = async () => {
    const router = useRouter();
    let accessToken = localStorage.getItem("accessToken");
    const refreshToken = localStorage.getItem("refreshToken");

    if (!accessToken) {
        alert("로그인이 필요합니다.");
        router.push("/mobile/login");
        return null;
    }

    try {
        // 1. 액세스 토큰 유효성 확인
        const validateResponse = await fetch(API_ROUTES.auth.validate, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ token: accessToken }),
        });

        const validateData = await validateResponse.json();

        if (validateResponse.ok && validateData.data.isValid) {
            // 토큰이 유효함
            return accessToken;
        }

        // 2. 토큰이 만료되었거나 유효하지 않은 경우, 리프레시 토큰으로 새 액세스 토큰 발급
        if (refreshToken) {
            const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ token: refreshToken }),
            });

            const refreshData = await refreshResponse.json();

            if (refreshResponse.ok && refreshData.data?.accessToken) {
                accessToken = refreshData.data.accessToken;
                if (accessToken) {
                    localStorage.setItem("accessToken", accessToken);
                }
                return accessToken;
            } else {
                alert("다시 로그인 해주세요.");
                localStorage.removeItem("accessToken");
                localStorage.removeItem("refreshToken");
                router.push("/mobile/login");
                return null;
            }
        } else {
            alert("다시 로그인 해주세요.");
            localStorage.removeItem("accessToken");
            router.push("/mobile/login");
            return null;
        }
    } catch (error) {
        console.error("토큰 검증 오류:", error);
        alert("인증 과정에서 오류가 발생했습니다.");
        return null;
    }
};
