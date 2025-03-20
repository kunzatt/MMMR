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

interface Module {
    name: string;
    component: React.ComponentType<any>; // 모든 컴포넌트 타입을 받도록 함
}

const verticalModules: Module[] = [
    { name: 'time', component: Time },
    { name: 'weather', component: Weather },
    { name: 'transportation', component: Transportation },
    { name: 'schedule', component: Schedule },
    { name: 'todo', component: Todo },
    { name: 'youtubev', component: Youtubev },
];

const horizontalModules: Module[] = [
    { name: 'news', component: News },
    { name: 'youtubeh', component: Youtubeh },
    { name: 'timer', component: Timer },
    { name: 'iot', component: Iot },
];

export default function Page() {
    const [activeModules, setActiveModules] = useState<Record<string, boolean>>({});
    const [isDarkMode, setIsDarkMode] = useState(false);
    const verticalContainerRef = useRef<HTMLDivElement>(null);
    const buttonContainerRef = useRef<HTMLDivElement>(null);
    const [availableHeight, setAvailableHeight] = useState<number>(0);
    const [prevActiveModules, setPrevActiveModules] = useState<Record<string, boolean>>({});

    // 화면 높이 계산 (버튼 높이 포함)
    useEffect(() => {
        const updateAvailableHeight = () => {
            const viewportHeight = window.innerHeight;
            const buttonHeight = buttonContainerRef.current?.offsetHeight || 0;
            const padding = 20;
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

    useEffect(() => {
        if (Object.keys(prevActiveModules).length === 0) return;

        const newUsedHeight = getUsedHeight();
        if (newUsedHeight > availableHeight) {
            alert('세로형 모듈이 화면을 초과하여 더 이상 추가할 수 없습니다.');
            setActiveModules(prevActiveModules);
        }
    }, [activeModules, availableHeight]);

    const toggleModule = (name: string) => {
        setPrevActiveModules(activeModules);
        setActiveModules((prev) => ({ ...prev, [name]: !prev[name] }));
    };

    const removeModule = (name: string) => {
        setActiveModules((prev) => {
            const updatedModules = { ...prev };
            delete updatedModules[name];
            return updatedModules;
        });
    };

    const toggleDarkMode = () => {
        setIsDarkMode(!isDarkMode);
    };

    return (
        <div
            className={`${
                isDarkMode ? 'bg-gray-300 text-white' : 'bg-gray-100 text-black'
            } font-sans flex flex-col items-center min-h-screen`}
        >
            {/* 다크 모드 토글 버튼 */}
            <button
                onClick={toggleDarkMode}
                className={`px-4 py-2 rounded-md mb-4 ${
                    isDarkMode ? 'bg-gray-600 text-white' : 'bg-gray-300 text-black'
                }`}
            >
                {isDarkMode ? 'Light Mode' : 'Dark Mode'}
            </button>

            {/* 버튼 UI */}
            <div ref={buttonContainerRef} className="flex flex-wrap gap-2 mb-6">
                {[...verticalModules, ...horizontalModules].map(({ name }) => (
                    <button
                        key={name}
                        onClick={() => toggleModule(name)}
                        className={`px-2 py-2 rounded-md ${
                            activeModules[name] ? 'bg-blue-500 text-white' : 'bg-blue-100 hover:bg-blue-200'
                        }`}
                    >
                        {activeModules[name] ? `${name}` : `${name}`}
                    </button>
                ))}
            </div>

            {/* 스택 UI */}
            <div className="flex w-full px-5 gap-4">
                {/* 세로 스택 */}
                <div
                    ref={verticalContainerRef}
                    className={`flex flex-col w-56 items-center overflow-hidden ${
                        isDarkMode ? 'border-white' : 'border-black'
                    }`}
                    style={{ maxHeight: `${availableHeight}px` }}
                >
                    {verticalModules
                        .filter(({ name }) => activeModules[name])
                        .map(({ name, component: Component }) => (
                            <div key={name} id={`module-${name}`}>
                                <Component isDarkMode={isDarkMode} />
                            </div>
                        ))}
                </div>
                {/* 가로 스택 */}
                <div className="flex flex-wrap w-full h-min justify-end items-start">
                    {horizontalModules.map(({ name, component: Component }) =>
                        activeModules[name] ? (
                            <div key={name} className="w-auto">
                                {name === 'timer' ? (
                                    <Timer onExpire={() => removeModule('timer')} />
                                ) : (
                                    <Component isDarkMode={isDarkMode} />
                                )}
                            </div>
                        ) : null
                    )}
                </div>
            </div>
        </div>
    );
}
