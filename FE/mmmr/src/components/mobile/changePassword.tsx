"use client";
import { useState } from "react";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";

interface ChangePasswordProps {
    onClose: () => void;
}

export default function ChangePassword({ onClose }: ChangePasswordProps) {
    const [currentPassword, setCurrentPassword] = useState("");
    const [newPassword, setNewPassword] = useState("");
    const [confirmPassword, setConfirmPassword] = useState("");
    const [showPassword, setShowPassword] = useState(false);

    const handleSubmit = () => {
        if (newPassword !== confirmPassword) {
            alert("비밀번호가 일치하지 않습니다.");
            return;
        }

        // 비밀번호 변경 API 호출 로직 추가
        alert("비밀번호가 성공적으로 변경되었습니다.");
        onClose();
    };

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-[80%] max-w-sm rounded-lg p-6 shadow-lg relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-6">비밀번호 변경</h2>

                {["현재 비밀번호", "비밀번호", "비밀번호 확인"].map((label, index) => (
                    <div>
                        <label key={index} className="block text-sm mb-1 text-gray-500">
                            {label}
                        </label>
                        <div key={index} className="relative mb-4">
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
                    ✖️
                </button>
            </div>
        </div>
    );
}
