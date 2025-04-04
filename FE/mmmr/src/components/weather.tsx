"use client";

import { JSX, useEffect, useState } from "react";
import { WiHumidity, WiRain } from "react-icons/wi";
import {
    TiWeatherCloudy,
    TiWeatherDownpour,
    TiWeatherPartlySunny,
    TiWeatherSnow,
    TiWeatherStormy,
    TiWeatherSunny,
    TiWeatherWindyCloudy
} from "react-icons/ti";
import { getToken } from "@/config/getToken";
import API_ROUTES from "@/config/apiRoutes";

const weatherIconMap: Record<string, JSX.Element> = {
    "흐림": <TiWeatherCloudy className="text-5xl" />,
    "비": <TiWeatherDownpour className="text-5xl" />,
    "약간 흐림": <TiWeatherPartlySunny className="text-5xl" />,
    "눈": <TiWeatherSnow className="text-5xl" />,
    "태풍": <TiWeatherStormy className="text-5xl" />,
    "맑음": <TiWeatherSunny className="text-5xl" />,
    "바람": <TiWeatherWindyCloudy className="text-5xl" />
};

interface WeatherData {
    currentTemperature: number;
    minTemperature: number;
    maxTemperature: number;
    humidity: number;
    precipitation: number;
    currentWeather: string; // 이 값으로 아이콘 선택
    clothingAdvice: string;
    umbrellaAdvice: string;
}

export default function Weather() {
    const [weather, setWeather] = useState<WeatherData | null>(null);

    useEffect(() => {
        const fetchWeather = async () => {
            try {
                const accessToken = await getToken();
                const response = await fetch(API_ROUTES.weather, {
                    method: "GET",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    }
                });

                if (response.ok) {
                    const data = await response.json();
                    setWeather(data.data);
                } else {
                    console.error("API 응답 오류:", response.status);
                }
            } catch (error) {
                console.error("날씨 불러오기 실패", error);
            }
        };

        fetchWeather();
    }, []);

    if (!weather) return <div className="text-center text-gray-500">날씨 정보를 불러오는 중...</div>;

    return (
        <div className="py-3 px-5 w-52 text-center flex flex-col gap-1">
            {/* 현재 기온 */}
            <div>
                <div className="flex justify-center items-center space-x-2">
                    {weatherIconMap[weather.currentWeather]}
                    <span className="text-4xl font-bold">{weather.currentTemperature}°</span>
                </div>

                {/* 최저/최고 기온 */}
                <div className="flex justify-center space-x-6 text-lg">
                    <span>{weather.minTemperature}°</span>
                    <span>{weather.maxTemperature}°</span>
                </div>
            </div>
            {/* 습도 & 강수 확률 */}
            <div className="flex justify-center items-center gap-2 text-md font-semibold">
                <div className="flex items-center">
                    <WiHumidity className="text-4xl" />
                    <span>Hum {weather.humidity}%</span>
                </div>
                <span>|</span>
                <div className="flex items-center">
                    <WiRain className="text-4xl" />
                    <span>Rain {weather.precipitation}%</span>
                </div>
            </div>

            {/* 날씨 설명 */}
            <p className="text-sm font-medium leading-relaxed break-keep whitespace-pre-line">
                {weather.clothingAdvice}
                {weather.umbrellaAdvice}
            </p>
        </div>
    );
}
