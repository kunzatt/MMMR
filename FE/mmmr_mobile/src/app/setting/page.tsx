"use client";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import ChangePassword from "@/components/changePassword";
import DeleteAccount from "@/components/deleteAccount";
import API_ROUTES from "@/config/apiRoutes";
import ShowQr from "@/components/showQr";

export default function Setting() {
    const router = useRouter();
    const [showChangePasswordModal, setShowChangePasswordModal] = useState(false);
    const [showDeleteAccountModal, setShowDeleteAccountModal] = useState(false);
    const [showQrModal, setShowQrModal] = useState(false);

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken");
            if (token) {
                const profile = localStorage.getItem("currentProfile");
                if (!profile) {
                    router.push("/profile");
                }
            } else {
                router.push("/login");
            }
        }
    }, [router]);

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/login");
            return null;
        }

        try {
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`
                },
                body: JSON.stringify({ token: accessToken })
            });

            if (!validateResponse.ok && refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    },
                    body: JSON.stringify({ token: refreshToken })
                });

                const refreshData = await refreshResponse.json();
                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    localStorage.setItem("accessToken", accessToken!);
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/login");
                    return null;
                }
            }

            return accessToken;
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const logout = async () => {
        const accessToken = await getTokens();

        if (!accessToken) {
            alert("이미 로그아웃된 상태입니다.");
            sessionStorage.setItem("hasReloaded", "true");

            router.push("/login");
            return;
        }

        try {
            const response = await fetch(API_ROUTES.auth.logout, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`
                },
                body: JSON.stringify({ token: accessToken })
            });

            const data = await response.json();

            if (response.ok) {
                alert(data.message || "로그아웃 성공");

                localStorage.removeItem("accessToken");
                localStorage.removeItem("refreshToken");
                localStorage.removeItem("currentProfile");
                sessionStorage.setItem("hasReloaded", "true");

                router.push("/login");
            } else {
                alert(data.message || "로그아웃 실패");
                console.error("로그아웃 실패:", data);
            }
        } catch (err) {
            console.error("로그아웃 오류:", err);
            alert("서버 오류가 발생했습니다.");
        }
    };

    return (
        <div className="flex-1 w-full flex h-full items-center justify-center relative">
            <div className="h-full w-full flex flex-col items-center justify-center p-6">
                <div className="flex flex-col h-full space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                    <h1 className="pl-2 pb-1 text-xl text-blue-300 font-bold border-b-2 border-blue-300">Setting</h1>
                    <button
                        className="w-full bg-blue-100 text-gray-600 py-2 px-4 rounded-full"
                        onClick={() => setShowChangePasswordModal(true)}
                    >
                        비밀번호 변경
                    </button>
                    <button
                        className="w-full bg-blue-100 text-gray-600 py-2 px-4 rounded-full"
                        onClick={() => router.push("/profile")}
                    >
                        프로필 관리
                    </button>
                    <button
                        className="w-full bg-blue-100 text-gray-600 py-2 px-4 rounded-full"
                        onClick={() => setShowQrModal(true)}
                    >
                        로그인 QR 코드
                    </button>
                    <button
                        className="w-full bg-blue-100 text-gray-600 py-2 px-4 rounded-full"
                        onClick={() => setShowDeleteAccountModal(true)}
                    >
                        회원 탈퇴
                    </button>
                    <button className="w-full bg-blue-100 text-gray-600 py-2 px-4 rounded-full" onClick={logout}>
                        로그아웃
                    </button>
                </div>
            </div>
            {showChangePasswordModal && <ChangePassword onClose={() => setShowChangePasswordModal(false)} />}
            {showDeleteAccountModal && <DeleteAccount onClose={() => setShowDeleteAccountModal(false)} />}
            {showQrModal && <ShowQr onClose={() => setShowQrModal(false)} />}
        </div>
    );
}
