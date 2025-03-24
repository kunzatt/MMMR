'use client';

import { WiDaySunny, WiHumidity, WiRain } from 'react-icons/wi';

const weatherData = {
    temperature: 27,
    minTemp: 20,
    maxTemp: 28,
    humidity: 54,
    rainProbability: 0.2,
    description: '오늘 날씨가 어떠니\n어떤걸 입거나 챙기세요',
};

export default function Weather() {
    return (
        <div className="py-3 px-5 w-52 text-center flex flex-col gap-1">
            {/* 현재 기온 */}
            <div>
                <div className="flex justify-center items-center space-x-2">
                    <WiDaySunny className="text-5xl" />
                    <span className="text-4xl font-bold">{weatherData.temperature}°</span>
                </div>

                {/* 최저/최고 기온 */}
                <div className="flex justify-center space-x-6 text-lg">
                    <span>{weatherData.minTemp}°</span>
                    <span>{weatherData.maxTemp}°</span>
                </div>
            </div>
            {/* 습도 & 강수 확률 */}
            <div className="flex justify-center items-center gap-2 text-md font-semibold">
                <div className="flex items-center">
                    <WiHumidity className="text-4xl" />
                    <span>Hum {weatherData.humidity}%</span>
                </div>
                <span>|</span>
                <div className="flex items-center">
                    <WiRain className="text-4xl" />
                    <span>Rain {weatherData.rainProbability}%</span>
                </div>
            </div>

            {/* 날씨 설명 */}
            <p className="text-sm font-medium leading-relaxed whitespace-pre-line">{weatherData.description}</p>
        </div>
    );
}
