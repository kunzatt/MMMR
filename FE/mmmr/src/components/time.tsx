'use client';

import { useState, useEffect } from 'react';

export default function Time() {
    const [currentTime, setCurrentTime] = useState<string>('');
    const [currentDate, setCurrentDate] = useState<string>('');

    useEffect(() => {
        // 시간을 업데이트하는 함수
        const updateTime = () => {
            const now = new Date();

            // 시간 HH:MM 형식 (24시간제)
            const hours = String(now.getHours()).padStart(2, '0');
            const minutes = String(now.getMinutes()).padStart(2, '0');
            setCurrentTime(`${hours}:${minutes}`);

            // 날짜 YYYY.MM.DD Day 형식
            const year = now.getFullYear();
            const month = String(now.getMonth() + 1).padStart(2, '0'); // 월 (0부터 시작)
            const day = String(now.getDate()).padStart(2, '0'); // 일
            const weekDay = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'][now.getDay()];
            setCurrentDate(`${year}.${month}.${day} ${weekDay}`);
        };

        updateTime(); // 초기 렌더링 시 바로 시간 표시
        const interval = setInterval(updateTime, 1000); // 1초마다 업데이트

        return () => clearInterval(interval); // 컴포넌트 언마운트 시 인터벌 정리
    }, []);

    return (
        <div className="py-3 px-5 h-auto w-48 shadow-md text-center">
            <p className="text-3xl font-bold pb-1">{currentTime}</p>
            <p className="text-xl">{currentDate}</p>
        </div>
    );
}
