import API_ROUTES from "./apiRoutes";

export const getToken = async () => {
    let accessToken = localStorage.getItem("accessToken");
    const refreshToken = localStorage.getItem("refreshToken");

    if (!accessToken) {
        return null;
    }

    try {
        const validateResponse = await fetch(API_ROUTES.auth.validate, {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
                "Authorization": `Bearer ${accessToken}`,
            },
            body: JSON.stringify({ token: accessToken }),
        });

        if (!validateResponse.ok && refreshToken) {
            const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ token: refreshToken }),
            });

            const refreshData = await refreshResponse.json();
            if (refreshResponse.ok && refreshData.data?.accessToken) {
                accessToken = refreshData.data.accessToken;
                localStorage.setItem("accessToken", accessToken!);
                return accessToken;
            } else {
                localStorage.removeItem("accessToken");
                localStorage.removeItem("refreshToken");
                return null;
            }
        }

        return accessToken;
    } catch (error) {
        console.error("토큰 검증 오류:", error);
        return null;
    }
};
