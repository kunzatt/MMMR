import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { AiOutlineEye, AiOutlineEyeInvisible } from 'react-icons/ai';

export default function LoginPage() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [showPassword, setShowPassword] = useState(false);
    const navigate = useNavigate();

    const handleLogin = () => {
        if (email === 'test@test.com' && password === 'password') {
            // 예제: 실제로는 서버 요청을 통해 확인해야 함
            localStorage.setItem('token', 'sampleToken');
            navigate('/mobile');
            alert('로그인 성공!');
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

                <div className="flex justify-between mt-2">
                    <span
                        onClick={() => navigate('/mobile/reset-password')}
                        className="text-sm text-blue-500 cursor-pointer"
                    >
                        비밀번호 찾기
                    </span>
                    <span onClick={() => navigate('/mobile/signup')} className="text-sm text-blue-500 cursor-pointer">
                        회원가입
                    </span>
                </div>
            </div>
        </div>
    );
}
