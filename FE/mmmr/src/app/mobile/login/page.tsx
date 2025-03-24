'use client';

import { useState } from 'react';
import { useRouter } from 'next/navigation';
import { AiOutlineEye, AiOutlineEyeInvisible } from 'react-icons/ai';

export default function LoginPage() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [showPassword, setShowPassword] = useState(false);
    const router = useRouter();

    const handleLogin = () => {
        if (email === 'test@test.com' && password === 'password') {
            localStorage.setItem('token', 'sampleToken');
            alert('로그인 성공!');
            router.refresh(); // 페이지를 새로고침하여 상태를 업데이트
            router.push('/mobile/home');
        } else {
            alert('로그인 실패! 이메일 또는 비밀번호를 확인하세요.');
        }
    };

    return (
        <div className="flex flex-col items-center justify-center h-full w-full">
            <div className="w-11/12 bg-white rounded-xl p-6 shadow-md space-y-4">
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
                    <label className="block text-sm mb-1 text-gray-500">PASSWORD</label>
                    <div className="flex items-center border rounded-md">
                        <input
                            className="w-full p-2"
                            type={showPassword ? 'text' : 'password'}
                            value={password}
                            onChange={(e) => setPassword(e.target.value)}
                            placeholder="********"
                        />
                        <div className="p-2 cursor-pointer" onClick={() => setShowPassword(!showPassword)}>
                            {showPassword ? <AiOutlineEye /> : <AiOutlineEyeInvisible />}
                        </div>
                    </div>
                </div>

                <button onClick={handleLogin} className="w-full bg-blue-300 text-white py-2 rounded-md mt-4">
                    Login
                </button>
            </div>
        </div>
    );
}
