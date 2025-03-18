'use client';

import { useState, useEffect, useRef } from 'react';
import Time from '@/components/time';
import Weather from '@/components/weather';
import Youtubeh from '@/components/youtubeh';
import Youtubev from '@/components/youtubev';
import Schedule from '@/components/schedule';
import Transportation from '@/components/transportation';
import Iot from '@/components/iot';
import Todo from '@/components/todo';
import Timer from '@/components/timer';
import News from '@/components/news';

const verticalModules = [
    { name: 'time', component: Time },
    { name: 'weather', component: Weather },
    { name: 'transportation', component: Transportation },
    { name: 'schedule', component: Schedule },
    { name: 'todo', component: Todo },
    { name: 'iot', component: Iot },
    { name: 'youtubev', component: Youtubev },
];

const horizontalModules = [
    { name: 'timer', component: Timer },
    { name: 'youtubeh', component: Youtubeh },
    { name: 'news', component: News },
];

export default function Page() {
    const [activeModules, setActiveModules] = useState<Record<string, boolean>>({});
    const verticalContainerRef = useRef<HTMLDivElement>(null);
    const buttonContainerRef = useRef<HTMLDivElement>(null);
    const [availableHeight, setAvailableHeight] = useState<number>(0);
    const [prevActiveModules, setPrevActiveModules] = useState<Record<string, boolean>>({});

    // 화면 높이 계산 (버튼 높이 포함)
    useEffect(() => {
        const updateAvailableHeight = () => {
            const viewportHeight = window.innerHeight;
            const buttonHeight = buttonContainerRef.current?.offsetHeight || 0;
            const padding = 20; // 여백 고려
            setAvailableHeight(viewportHeight - buttonHeight - padding);
        };

        updateAvailableHeight();
        window.addEventListener('resize', updateAvailableHeight);
        return () => window.removeEventListener('resize', updateAvailableHeight);
    }, []);

    // 현재 사용 중인 세로 모듈의 높이를 동적으로 측정
    const getUsedHeight = () => {
        if (!verticalContainerRef.current) return 0;
        return Array.from(verticalContainerRef.current.children).reduce(
            (sum, child) => sum + (child as HTMLElement).offsetHeight,
            0
        );
    };

    // 모듈이 추가된 후 높이를 체크하고 초과하면 원래 상태로 되돌리기
    useEffect(() => {
        if (Object.keys(prevActiveModules).length === 0) return;

        const newUsedHeight = getUsedHeight();
        if (newUsedHeight > availableHeight) {
            alert('세로형 모듈이 화면을 초과하여 더 이상 추가할 수 없습니다.');
            setActiveModules(prevActiveModules); // 이전 상태로 복구
        }
    }, [activeModules, availableHeight]);

    // 모듈 추가 및 삭제 함수
    const toggleModule = (name: string) => {
        setPrevActiveModules(activeModules); // 이전 상태 저장
        setActiveModules((prev) => ({ ...prev, [name]: !prev[name] }));
    };

    const removeModule = (name: string) => {
        setActiveModules((prev) => {
            const updatedModules = { ...prev };
            delete updatedModules[name];
            return updatedModules;
        });
    };

    return (
        <div className="font-sans flex flex-col items-center min-h-screen">
            {/* 버튼 UI (높이 고려) */}
            <div ref={buttonContainerRef} className="flex flex-wrap gap-2 mb-6">
                {[...verticalModules, ...horizontalModules].map(({ name }) => (
                    <button
                        key={name}
                        onClick={() => toggleModule(name)}
                        className={`px-2 py-2 rounded-md ${
                            activeModules[name] ? 'bg-red-500 text-white' : 'bg-blue-100 hover:bg-blue-200'
                        }`}
                    >
                        {activeModules[name] ? `${name}` : `${name}`}
                    </button>
                ))}
            </div>

            {/* 스택 UI */}
            <div className="flex w-full px-5 gap-4">
                {/* 세로 스택 (버튼 높이를 고려한 최대 높이 제한) */}
                <div
                    ref={verticalContainerRef}
                    className="flex flex-col gap-4 w-48 items-center overflow-hidden"
                    style={{ maxHeight: `${availableHeight}px` }}
                >
                    {verticalModules
                        .filter(({ name }) => activeModules[name])
                        .map(({ name, component: Component }) => (
                            <div key={name} id={`module-${name}`}>
                                <Component />
                            </div>
                        ))}
                </div>

                {/* 가로 스택 (정해진 순서 유지) */}
                <div className="flex flex-wrap gap-4 w-full h-min justify-end items-start">
                    {horizontalModules.map(({ name, component: Component }) =>
                        activeModules[name] ? (
                            <div
                                key={name}
                                className="w-auto"
                                style={{
                                    order: name === 'news' ? -1 : 0, // 뉴스는 항상 아래로 내려가지 않도록 유지
                                    alignSelf: 'flex-start', // 모듈들이 위쪽에 고정됨
                                }}
                            >
                                {name === 'timer' ? (
                                    <Timer onExpire={() => removeModule('timer')} /> // 👈 여기만 추가
                                ) : (
                                    <Component />
                                )}
                            </div>
                        ) : null
                    )}
                </div>
            </div>
        </div>
    );
}
