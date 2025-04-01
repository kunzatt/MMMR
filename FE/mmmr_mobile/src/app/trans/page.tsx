"use client";
//import API_ROUTES from "@/config/apiRoutes";
import { useRouter } from "next/navigation";
import { useEffect } from "react";

export default function Trans() {
    const router = useRouter();

    // const getTokens = async () => {
    //     let accessToken = localStorage.getItem("accessToken");
    //     const refreshToken = localStorage.getItem("refreshToken");

    //     if (!accessToken) {
    //         router.push("/login");
    //         return null;
    //     }

    //     try {
    //         // 1. 액세스 토큰 유효성 확인
    //         const validateResponse = await fetch(API_ROUTES.auth.validate, {
    //             method: "POST",
    //             headers: {
    //                 "Content-Type": "application/json",
    //                 "Authorization": `Bearer ${refreshToken}`,
    //             },
    //             body: JSON.stringify({ token: accessToken }),
    //         });

    //         let validateData = null;
    //         if (validateResponse.ok) {
    //             const contentType = validateResponse.headers.get("Content-Type");
    //             if (contentType && contentType.includes("application/json")) {
    //                 validateData = await validateResponse.json();
    //             }
    //         }

    //         // 2. 토큰이 만료되었거나 유효하지 않은 경우, 리프레시 토큰으로 새 액세스 토큰 발급
    //         if (refreshToken) {
    //             const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
    //                 method: "POST",
    //                 headers: {
    //                     "Content-Type": "application/json",
    //                     "Authorization": `Bearer ${refreshToken}`,
    //                 },
    //                 body: JSON.stringify({ token: refreshToken }),
    //             });

    //             let refreshData = null;
    //             if (refreshResponse.ok) {
    //                 const contentType = refreshResponse.headers.get("Content-Type");
    //                 if (contentType && contentType.includes("application/json")) {
    //                     refreshData = await refreshResponse.json();
    //                 }
    //             }

    //             if (refreshResponse.ok && refreshData.data?.accessToken) {
    //                 accessToken = refreshData.data.accessToken;
    //                 if (accessToken) {
    //                     localStorage.setItem("accessToken", accessToken);
    //                 }
    //                 return accessToken;
    //             } else {
    //                 localStorage.removeItem("accessToken");
    //                 localStorage.removeItem("refreshToken");
    //                 router.push("/login");
    //                 return null;
    //             }
    //         } else {
    //             localStorage.removeItem("accessToken");
    //             router.push("/login");
    //             return null;
    //         }
    //     } catch (error) {
    //         console.error("토큰 검증 오류:", error);
    //         return null;
    //     }
    // };

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken"); // 'token' -> 'accessToken'으로 수정
            if (token) {
                const profile = localStorage.getItem("currentProfile"); // 'profile' -> 'currentProfile'으로 수정
                if (!profile) {
                    router.push("/profile"); // 프로필이 없으면 프로필 페이지로 리다이렉트
                }
            } else {
                router.push("/login"); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
    }, [router]);

    return (
        <div className="flex-1 w-full flex h-full items-center justify-center relative">
            <div className="h-full w-full flex flex-col items-center justify-center p-6">
                <div className="flex flex-col h-full items-center space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                    <h1 className="text-xl text-blue-300 font-bold">Transportation</h1>
                </div>
            </div>
        </div>
    );
}
