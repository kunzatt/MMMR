"use client";
import { useState } from "react";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";
import { ImCross } from "react-icons/im";
import { API_ROUTES } from "@/config/apiRoutes";
import { useRouter } from "next/navigation";

interface ChangePasswordProps {
    onClose: () => void;
}

export default function ChangePassword({ onClose }: ChangePasswordProps) {
    const [currentPassword, setCurrentPassword] = useState("");
    const [newPassword, setNewPassword] = useState("");
    const [confirmPassword, setConfirmPassword] = useState("");
    const [showPassword, setShowPassword] = useState(false);

    const router = useRouter();

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/mobile/login");
            return null;
        }

        try {
            // 1. 액세스 토큰 유효성 확인
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${refreshToken}`,
                },
                body: JSON.stringify({ token: accessToken }),
            });

            let validateData = null;
            if (validateResponse.ok) {
                const contentType = validateResponse.headers.get("Content-Type");
                if (contentType && contentType.includes("application/json")) {
                    validateData = await validateResponse.json();
                }
            }

            // 2. 토큰이 만료되었거나 유효하지 않은 경우, 리프레시 토큰으로 새 액세스 토큰 발급
            if (refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${refreshToken}`,
                    },
                    body: JSON.stringify({ token: refreshToken }),
                });

                let refreshData = null;
                if (refreshResponse.ok) {
                    const contentType = refreshResponse.headers.get("Content-Type");
                    if (contentType && contentType.includes("application/json")) {
                        refreshData = await refreshResponse.json();
                    }
                }

                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    if (accessToken) {
                        localStorage.setItem("accessToken", accessToken);
                    }
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/mobile/login");
                    return null;
                }
            } else {
                localStorage.removeItem("accessToken");
                router.push("/mobile/login");
                return null;
            }
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const handleSubmit = async () => {
        if (newPassword !== confirmPassword) {
            alert("비밀번호가 일치하지 않습니다.");
            return;
        }

        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.accounts.changePassword, {
                method: "PUT",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({
                    currentPassword: currentPassword,
                    newPassword: newPassword,
                    newPasswordConfirm: confirmPassword,
                }),
            });

            const data = await response.json();

            if (response.ok) {
                alert(data.message || "비밀번호가 성공적으로 변경되었습니다.");
                onClose();
            } else {
                alert(data.message || "비밀번호 변경에 실패했습니다.");
            }
        } catch (error) {
            console.error("비밀번호 변경 오류:", error);
            alert("비밀번호 변경 중 오류가 발생했습니다.");
        }
    };

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-[80%] max-w-sm rounded-lg p-6 shadow-lg relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-6">비밀번호 변경</h2>

                {["현재 비밀번호", "비밀번호", "비밀번호 확인"].map((label, index) => (
                    <div key={index}>
                        <label className="block text-sm mb-1 text-gray-500">{label}</label>
                        <div className="relative mb-4">
                            <input
                                type={showPassword ? "text" : "password"}
                                placeholder="********"
                                value={index === 0 ? currentPassword : index === 1 ? newPassword : confirmPassword}
                                onChange={(e) => {
                                    if (index === 0) setCurrentPassword(e.target.value);
                                    if (index === 1) setNewPassword(e.target.value);
                                    if (index === 2) setConfirmPassword(e.target.value);
                                }}
                                className="w-full p-2 border rounded-md"
                            />
                            <div
                                className="absolute right-2 top-1/2 transform -translate-y-1/2 cursor-pointer"
                                onClick={() => setShowPassword(!showPassword)}
                            >
                                {showPassword ? <AiOutlineEye /> : <AiOutlineEyeInvisible />}
                            </div>
                        </div>
                    </div>
                ))}

                <button className="w-full bg-blue-300 text-white py-2 px-4 rounded-md mt-4" onClick={handleSubmit}>
                    Change
                </button>
                <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                    <ImCross />
                </button>
            </div>
        </div>
    );
}
