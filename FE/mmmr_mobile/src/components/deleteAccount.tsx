"use client";
import { useState } from "react";
import { useRouter } from "next/navigation";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";
import { ImCross } from "react-icons/im";
import API_ROUTES from "@/config/apiRoutes";

interface DeleteAccountProps {
    onClose: () => void;
}

export default function DeleteAccount({ onClose }: DeleteAccountProps) {
    const [password, setPassword] = useState("");
    const [showPassword, setShowPassword] = useState(false);
    const router = useRouter();

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

    const handleDeleteAccount = async () => {
        if (!password) {
            alert("비밀번호를 입력해주세요.");
            return;
        }
        const accessToken = await getTokens();
        if (!accessToken) return;
        // 여기서 API 호출로 회원 탈퇴 요청을 보냅니다.
        try {
            const response = await fetch("/api/accounts/delete", {
                // API 경로 확인 필요
                method: "DELETE",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ password }),
            });

            if (response.ok) {
                alert("회원 탈퇴가 완료되었습니다.");
                localStorage.removeItem("accessToken");
                localStorage.removeItem("currentProfile");
                router.push("/login");
            } else {
                const result = await response.json();
                alert(result.message || "회원 탈퇴 실패. 비밀번호를 다시 확인해주세요.");
            }
        } catch (error) {
            console.error("회원 탈퇴 오류:", error);
        }
    };

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-sm rounded-lg p-6 relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-4">회원 탈퇴</h2>
                <div>
                    <label className="block text-sm text-gray-500 mb-1">현재 비밀번호</label>

                    <div className="mb-4 relative">
                        <input
                            type={showPassword ? "text" : "password"}
                            value={password}
                            onChange={(e) => setPassword(e.target.value)}
                            className="w-full p-2 border rounded-md"
                            placeholder="********"
                        />
                        <div
                            className="absolute right-2 top-1/2 transform -translate-y-1/2 cursor-pointer"
                            onClick={() => setShowPassword(!showPassword)}
                        >
                            {showPassword ? <AiOutlineEyeInvisible /> : <AiOutlineEye />}
                        </div>
                    </div>
                </div>
                <p className="text-xs text-gray-500 mb-4">회원 탈퇴는 되돌릴 수 없습니다.</p>
                <div className="flex justify-center mt-4">
                    <button onClick={handleDeleteAccount} className="bg-blue-300 text-white py-2 px-4 rounded-md w-1/3">
                        탈퇴
                    </button>
                    <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                        <ImCross />
                    </button>
                </div>
            </div>
        </div>
    );
}
