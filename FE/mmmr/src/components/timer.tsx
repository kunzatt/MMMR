'use client';

import { useState, useEffect } from 'react';

export default function Timer() {
    const [timeLeft, setTimeLeft] = useState(10 * 60); // 초기값 (예: 10분)

    useEffect(() => {
        if (timeLeft <= 0) return; // 시간이 0이면 타이머 정지

        const timer = setInterval(() => {
            setTimeLeft((prev) => prev - 1);
        }, 1000);

        return () => clearInterval(timer); // 언마운트 시 정리
    }, [timeLeft]);

    // 시간 포맷 (MM:SS)
    const formatTime = (seconds: number) => {
        const minutes = Math.floor(seconds / 60)
            .toString()
            .padStart(2, '0');
        const secs = (seconds % 60).toString().padStart(2, '0');
        return `${minutes}:${secs}`;
    };
    return (
        <div className="py-3 px-10 w-auto h-44 shadow-md text-center flex items-center">
            <p className="text-7xl font-medium">{formatTime(timeLeft)}</p>
        </div>
    );
}
