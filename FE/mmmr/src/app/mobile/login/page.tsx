'use client';

import { useState } from 'react';
import { useRouter } from 'next/navigation';
import { AiOutlineEye, AiOutlineEyeInvisible } from 'react-icons/ai';
import { API_ROUTES } from '@/config/apiRoutes';

export default function LoginPage() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [showPassword, setShowPassword] = useState(false);
    const [loading, setLoading] = useState(false);
    const router = useRouter();

    const handleLogin = async () => {
        try {
            const response = await fetch(API_ROUTES.auth.login, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ email, password }),
            });

            const data = await response.json();

            if (response.ok) {
                alert('로그인 성공!');
                localStorage.setItem('accessToken', data.data.accessToken);
                localStorage.setItem('refreshToken', data.data.refreshToken);
                router.push('/mobile/home');
            } else {
                alert(data.message || '로그인에 실패했습니다. 다시 시도하세요.');
                console.log('로그인 실패:', data.message);
            }
        } catch (error) {
            alert('서버 오류가 발생했습니다. 다시 시도해주세요.');
            console.log('로그인 오류:', error);
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

                <button
                    onClick={handleLogin}
                    className="w-full bg-blue-300 text-white py-2 rounded-md mt-4"
                    disabled={loading}
                >
                    {loading ? '로그인 중...' : 'Login'}
                </button>

                <div className="flex justify-between mt-2">
                    <span
                        onClick={() => router.push('/mobile/reset-password')}
                        className="text-sm text-blue-500 cursor-pointer"
                    >
                        비밀번호 찾기
                    </span>
                    <span
                        onClick={() => router.push('/mobile/signup')}
                        className="text-sm text-blue-500 cursor-pointer"
                    >
                        회원가입
                    </span>
                </div>
            </div>
        </div>
    );
}
