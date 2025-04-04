"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";
import { API_ROUTES } from "@/config/apiRoutes";

export default function LoginPage() {
    const [email, setEmail] = useState("");
    const [password, setPassword] = useState("");
    const [showPassword, setShowPassword] = useState(false);
    const loading = false;
    const [showResetModal, setShowResetModal] = useState(false);
    const [resetEmail, setResetEmail] = useState("");
    const router = useRouter();

    const handleLogin = async () => {
        try {
            const response = await fetch(API_ROUTES.auth.login, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ email, password })
            });

            const data = await response.json();

            if (response.ok) {
                console.log("로그인 성공!");
                localStorage.setItem("email", email);
                localStorage.setItem("password", password);
                localStorage.setItem("accessToken", data.data.accessToken);
                localStorage.setItem("refreshToken", data.data.refreshToken);
                sessionStorage.setItem("hasReloaded", "true");
                router.push("/profile");
            } else {
                console.log(data.message || "로그인에 실패했습니다. 다시 시도하세요.");
                console.log("로그인 실패:", data.message);
            }
        } catch (error) {
            console.log("서버 오류가 발생했습니다. 다시 시도해주세요.");
            console.log("로그인 오류:", error);
        }
    };

    return (
        <div className="flex flex-col items-center justify-center h-full w-full">
            <div className="w-11/12 bg-white rounded-xl p-6 shadow-md space-y-4 relative z-10">
                <div>
                    <label className="block text-sm mb-1 text-gray-500">email</label>
                    <input
                        className="w-full p-2 border rounded-md"
                        type="email"
                        value={email}
                        onChange={(e) => setEmail(e.target.value)}
                        placeholder="login@email.com"
                    />
                </div>

                <div>
                    <label className="block text-sm mb-1 text-gray-500">password</label>
                    <div className="flex items-center border rounded-md">
                        <input
                            className="w-full p-2"
                            type={showPassword ? "text" : "password"}
                            value={password}
                            onChange={(e) => setPassword(e.target.value)}
                            placeholder="********"
                        />
                        <div className="p-2 cursor-pointer" onClick={() => setShowPassword(!showPassword)}>
                            {showPassword ? <AiOutlineEye /> : <AiOutlineEyeInvisible />}
                        </div>
                    </div>
                </div>

                <button
                    onClick={handleLogin}
                    className="w-full bg-blue-300 text-white py-2 rounded-md mt-4"
                    disabled={loading}
                >
                    {loading ? "로그인 중..." : "Login"}
                </button>

                <div className="flex justify-between mt-2">
                    <span onClick={() => setShowResetModal(true)} className="text-sm text-blue-500 cursor-pointer">
                        비밀번호 찾기
                    </span>
                    <span onClick={() => router.push("/signup")} className="text-sm text-blue-500 cursor-pointer">
                        회원가입
                    </span>
                </div>
            </div>

            {showResetModal && (
                <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
                    <div className="bg-white w-10/12 max-w-sm rounded-lg p-6">
                        <h2 className="text-lg font-medium text-center mb-4">비밀번호 초기화</h2>
                        <input
                            className="w-full p-2 border rounded-md mb-2"
                            type="email"
                            value={resetEmail}
                            onChange={(e) => setResetEmail(e.target.value)}
                            placeholder="login@email.com"
                        />
                        <p className="text-sm text-gray-500 mb-4">초기화된 비밀번호가 이메일로 발송됩니다.</p>
                        <div className="flex justify-between">
                            <button className="bg-blue-300 text-white py-2 px-4 rounded-md">Send Email</button>
                            <button
                                onClick={() => setShowResetModal(false)}
                                className="bg-gray-300 text-white py-2 px-4 rounded-md"
                            >
                                Cancel
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}
