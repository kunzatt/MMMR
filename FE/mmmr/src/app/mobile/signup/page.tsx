"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";
import API_ROUTES from "@/config/apiRoutes";

export default function SignupPage() {
    const [email, setEmail] = useState("");
    const [showConfirmEmail, setShowConfirmEmail] = useState(false);
    const [emailCode, setEmailCode] = useState("");
    const [password, setPassword] = useState("");
    const [confirmPassword, setConfirmPassword] = useState("");
    const [address, setAddress] = useState("");
    const [showPassword, setShowPassword] = useState(false);
    const [showConfirmPassword, setShowConfirmPassword] = useState(false);
    const [emailVerified, setEmailVerified] = useState(false);
    const [emailError, setEmailError] = useState("");
    const router = useRouter();

    const validateEmail = (email: string) => {
        const emailPattern = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
        return emailPattern.test(email);
    };

    const handleEmailCheck = async () => {
        if (!validateEmail(email)) {
            setEmailError("이메일 형식이 잘못되었습니다. 다시 입력해주세요.");
            return;
        }

        setEmailError(""); // 오류 메시지 초기화

        try {
            const response = await fetch(`${API_ROUTES.accounts.emailExists}?email=${email}`);
            const data = await response.json();

            if (response.ok && !data.data.exists) {
                alert("사용 가능한 이메일입니다.");
                setEmailVerified(true);
                try {
                    const response = await fetch(API_ROUTES.accounts.sendCodes, {
                        method: "POST",
                        body: JSON.stringify({ email: email }),
                    });
                    const data = await response.json();
                    if (response.ok) {
                        alert(data.message || "인증 코드가 전송되었습니다.");
                        setShowConfirmEmail(true);
                    } else {
                        alert(data.message || "인증 코드 전송에 실패했습니다.");
                    }
                } catch (error) {
                    alert("인증 코드 전송 오류가 발생했습니다. 다시 시도해주세요.");
                    console.error("인증 코드 전송 오류:", error);
                }
            } else {
                setEmailError(data.message || "이미 존재하는 이메일입니다.");
            }
        } catch (error) {
            alert("이메일 확인 오류가 발생했습니다. 다시 시도해주세요.");
            console.error("이메일 확인 오류:", error);
        }
    };

    const handleEmailVerification = async () => {
        try {
            const response = await fetch(API_ROUTES.accounts.codeVerification(email), {
                method: "POST",
                body: JSON.stringify({ code: emailCode }),
            });
            const data = await response.json();
            if (response.ok) {
                alert(data.message || "이메일 인증 성공!");
                setEmailVerified(true);
            } else {
                setEmailError(data.message || "이메일 인증에 실패했습니다.");
            }
        } catch (error) {
            alert("이메일 인증 오류가 발생했습니다. 다시 시도해주세요.");
            console.error("이메일 인증 오류:", error);
        }
    };

    const handleSignup = async () => {
        if (!emailVerified) {
            alert("이메일 인증을 먼저 완료하세요.");
            return;
        }

        if (password !== confirmPassword) {
            alert("비밀번호가 일치하지 않습니다.");
            return;
        }

        try {
            const response = await fetch(API_ROUTES.accounts.signUp, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
                body: JSON.stringify({
                    email: email,
                    password: password,
                    address: address,
                }),
            });

            // 서버 응답이 JSON 형식인지 확인
            let data;
            const responseText = await response.text(); // 먼저 text로 받기

            try {
                data = JSON.parse(responseText); // JSON으로 파싱 시도
            } catch (error) {
                console.error("JSON 파싱 오류:", error);
                console.error("서버 응답 원문:", responseText); // 서버의 응답 내용 확인
                alert("서버에서 예상치 못한 응답이 왔습니다.");
                return; // JSON 파싱 에러가 발생하면 함수 종료
            }

            if (response.ok) {
                alert(data.message || "회원가입 성공!");
                router.push("/mobile/login");
            } else {
                alert(data.message || "회원가입에 실패했습니다. 다시 시도하세요.");
                console.warn("회원가입 오류:", data);
            }
        } catch (error) {
            alert("서버 오류가 발생했습니다. 다시 시도해주세요.");
            console.error("회원가입 오류:", error);
        }
    };

    return (
        <div className="flex flex-col items-center justify-center h-full w-full">
            <div className="w-11/12 bg-white rounded-xl p-6 shadow-md space-y-4">
                <div className="flex flex-col">
                    <div className="flex">
                        <input
                            className={`w-full p-2 border rounded-md h-10 ${emailError ? "border-red-500" : ""}`}
                            type="email"
                            value={email}
                            onChange={(e) => setEmail(e.target.value)}
                            placeholder="email"
                        />
                        <button
                            onClick={handleEmailCheck}
                            className="bg-blue-300 text-white text-sm break-keep rounded-md ml-2 h-10 w-24 px-2"
                        >
                            이메일 인증
                        </button>
                    </div>
                    {emailError && <p className="text-red-500 text-sm mt-1">{emailError}</p>}
                </div>

                {showConfirmEmail && (
                    <div className="flex flex-col">
                        <div className="flex">
                            <input
                                className="w-full p-2 border rounded-md h-10"
                                type="text"
                                value={emailCode}
                                onChange={(e) => setEmailCode(e.target.value)}
                                placeholder="인증 코드"
                            />
                            <button
                                onClick={handleEmailVerification}
                                className="bg-blue-300 text-white text-sm break-keep rounded-md ml-2  h-10  w-24 px-2"
                            >
                                인증 코드 확인
                            </button>
                        </div>
                        {emailError && <p className="text-red-500 text-sm mt-1">{emailError}</p>}
                    </div>
                )}

                <div className="relative">
                    <input
                        className="w-full p-2 pl-3 border rounded-md h-10"
                        type={showPassword ? "text" : "password"}
                        value={password}
                        onChange={(e) => setPassword(e.target.value)}
                        placeholder="비밀번호"
                    />
                    <div
                        className="absolute right-2 top-1/2 transform -translate-y-1/2 cursor-pointer"
                        onClick={() => setShowPassword(!showPassword)}
                    >
                        {showPassword ? <AiOutlineEye /> : <AiOutlineEyeInvisible />}
                    </div>
                </div>

                <div className="relative mt-4">
                    <input
                        className="w-full p-2 pl-3 border rounded-md h-10"
                        type={showConfirmPassword ? "text" : "password"}
                        value={confirmPassword}
                        onChange={(e) => setConfirmPassword(e.target.value)}
                        placeholder="비밀번호 확인"
                    />
                    <div
                        className="absolute right-2 top-1/2 transform -translate-y-1/2 cursor-pointer"
                        onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                    >
                        {showConfirmPassword ? <AiOutlineEye /> : <AiOutlineEyeInvisible />}
                    </div>
                </div>

                <input
                    className="w-full p-2 border rounded-md h-10"
                    type="text"
                    value={address}
                    onChange={(e) => setAddress(e.target.value)}
                    placeholder="주소"
                />

                <button onClick={handleSignup} className="w-full bg-blue-300 text-white py-2 rounded-md mt-4 h-10">
                    Sign up
                </button>
            </div>
        </div>
    );
}
