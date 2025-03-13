'use client';

import { useState, useEffect } from 'react';

export default function Time() {
    const [currentTime, setCurrentTime] = useState<string>('');

    useEffect(() => {
        // 현재 시간을 업데이트하는 함수
        const updateTime = () => {
            const now = new Date();
            const formattedTime = now.toLocaleTimeString();
            setCurrentTime(formattedTime);
        };

        updateTime(); // 초기 렌더링 시 바로 시간 표시
        const interval = setInterval(updateTime, 1000); // 1초마다 업데이트

        return () => clearInterval(interval); // 컴포넌트 언마운트 시 인터벌 정리
    }, []);

    return (
        <div className="p-3 h-32 rounded-lg shadow-md text-center">
            <h2 className="text-xl font-semibold">🕒 Current Time</h2>
            <p className="text-2xl font-bold mt-2">{currentTime}</p>
        </div>
    );
}
