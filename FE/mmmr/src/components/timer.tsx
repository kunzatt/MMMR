'use client';

import { useState, useEffect } from 'react';

interface TimerProps {
    onExpire?: () => void; // 타이머 종료 시 실행할 콜백 함수
}

export default function Timer({ onExpire }: TimerProps) {
    const [timeLeft, setTimeLeft] = useState(1 * 60); // 초기값 (10분)

    useEffect(() => {
        if (timeLeft <= 0) {
            alert('타이머가 종료되었습니다.'); // 알림창 표시
            if (onExpire) onExpire(); // 스택에서 제거
            return;
        }

        const timer = setInterval(() => {
            setTimeLeft((prev) => prev - 1);
        }, 1000);

        return () => clearInterval(timer);
    }, [timeLeft, onExpire]);

    // 시간 포맷 (MM:SS)
    const formatTime = (seconds: number) => {
        const minutes = Math.floor(seconds / 60)
            .toString()
            .padStart(2, '0');
        const secs = (seconds % 60).toString().padStart(2, '0');
        return `${minutes}:${secs}`;
    };

    return (
        <div className="py-3 px-10 w-auto h-44 text-center flex items-center justify-center">
            <p className="text-7xl font-medium">{formatTime(timeLeft)}</p>
        </div>
    );
}
