'use client';

import { useState } from 'react';
import Time from '@/components/time';
import Weather from '@/components/weather';
import Youtube from '@/components/youtube';
import Schedule from '@/components/schedule';
import Transportation from '@/components/transportation';
import Iot from '@/components/iot';
import Todo from '@/components/todo';
import Timer from '@/components/timer';
import News from '@/components/news';

const vcomponentMap: Record<string, React.ComponentType> = {
    time: Time,
    weather: Weather,
    transportation: Transportation,
    schedule: Schedule,
    todo: Todo,
    iot: Iot,
};

const hcomponentMap: Record<string, React.ComponentType> = {
    timer: Timer,
    youtube: Youtube,
    news: News,
};

export default function Page() {
    const [verticalStack, setVerticalStack] = useState<string[]>([]);
    const [horizontalStack, setHorizontalStack] = useState<string[]>([]);

    // 특정 컴포넌트 추가 (중복 방지)
    const addToVerticalStack = (name: string) => {
        setVerticalStack((prev) => (prev.includes(name) ? prev : [...prev, name]));
    };

    const addToHorizontalStack = (name: string) => {
        setHorizontalStack((prev) => (prev.includes(name) ? prev : [name, ...prev]));
    };

    // 특정 컴포넌트 삭제
    const removeFromVerticalStack = (name: string) => {
        setVerticalStack((prev) => prev.filter((item) => item !== name));
    };

    const removeFromHorizontalStack = (name: string) => {
        setHorizontalStack((prev) => prev.filter((item) => item !== name));
    };

    return (
        <div className="flex flex-col items-center mt-10 min-h-screen">
            {/* 추가 버튼 그룹 */}
            <div className="flex flex-wrap gap-2 mb-6">
                {Object.keys(vcomponentMap).map((name) => (
                    <button
                        key={name}
                        onClick={() => addToVerticalStack(name)}
                        className="px-2 py-2 bg-blue-100 rounded-md hover:bg-blue-200"
                    >
                        Add {name}
                    </button>
                ))}
                {Object.keys(hcomponentMap).map((name) => (
                    <button
                        key={name}
                        onClick={() => addToHorizontalStack(name)}
                        className="px-2 py-2 bg-red-100 rounded-md hover:bg-red-200"
                    >
                        Add {name}
                    </button>
                ))}
            </div>

            {/* 삭제 버튼 그룹 */}
            <div className="flex flex-wrap gap-2 mb-6">
                {verticalStack.map((name) => (
                    <button
                        key={name}
                        onClick={() => removeFromVerticalStack(name)}
                        className="px-2 py-2 bg-blue-800 rounded-md hover:bg-blue-700 text-white"
                    >
                        Remove {name}
                    </button>
                ))}
                {horizontalStack.map((name) => (
                    <button
                        key={name}
                        onClick={() => removeFromHorizontalStack(name)}
                        className="px-2 py-2 bg-red-800 rounded-md hover:bg-red-700 text-white"
                    >
                        Remove {name}
                    </button>
                ))}
            </div>

            {/* 스택 UI */}
            <div className="flex w-full max-w-4xl gap-6">
                {/* 왼쪽: 세로 스택 */}
                <div className="flex flex-col gap-4 w-1/4">
                    {verticalStack.map((name, index) => {
                        const Component = vcomponentMap[name];
                        return Component ? (
                            <div key={index}>
                                <Component />
                            </div>
                        ) : null;
                    })}
                </div>

                {/* 오른쪽: 가로 스택 */}
                <div className="flex flex-wrap-reverse gap-4 w-3/4 justify-end">
                    {horizontalStack.map((name, index) => {
                        const Component = hcomponentMap[name];
                        return Component ? (
                            <div key={index}>
                                <Component />
                            </div>
                        ) : null;
                    })}
                </div>
            </div>
        </div>
    );
}
