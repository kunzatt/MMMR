"use client";

import { useState, useEffect, useRef } from "react";
import Time from "@/components/time";
import Weather from "@/components/weather";
import Youtubeh from "@/components/youtubeh";
import Youtubev from "@/components/youtubev";
import Schedule from "@/components/schedule";
import Transportation from "@/components/transportation";
import Iot from "@/components/iot";
import Todo from "@/components/todo";
import Timer from "@/components/timer";
import News from "@/components/news";

interface Module {
    name: string;
    component: React.ComponentType;
}

// type과 모듈 이름 사이의 매핑 정의
const moduleTypeMapping: Record<string, string> = {
    "time": "time",
    "weather": "weather",
    "transportation": "transportation",
    "schedule": "schedule",
    "todo": "todo",
    "youtube": "youtubev", // 기본적으로 세로형 유튜브 사용
    "timer": "timer",
    "news": "news",
    "iot": "iot",
};

const verticalModules: Module[] = [
    { name: "time", component: Time },
    { name: "weather", component: Weather },
    { name: "transportation", component: Transportation },
    { name: "schedule", component: Schedule },
    { name: "todo", component: Todo },
    { name: "youtubev", component: Youtubev },
];

const horizontalModules: Module[] = [
    { name: "news", component: News },
    { name: "youtubeh", component: Youtubeh },
    { name: "timer", component: Timer },
    { name: "iot", component: Iot },
];

// 웹소켓 메시지 타입 정의
interface WebSocketMessage {
    type: string;
    contents: {
        default: "ON" | "OFF";
        data: string;
    };
}

export default function Page() {
    const [activeModules, setActiveModules] = useState<Record<string, boolean>>({});
    const verticalContainerRef = useRef<HTMLDivElement>(null);
    const buttonContainerRef = useRef<HTMLDivElement>(null);
    const [availableHeight, setAvailableHeight] = useState<number>(0);
    const [prevActiveModules, setPrevActiveModules] = useState<Record<string, boolean>>({});

    // 웹소켓 참조 저장
    const webSocketRef = useRef<WebSocket | null>(null);

    // 웹소켓 연결 상태
    const [isConnected, setIsConnected] = useState(false);

    // 모듈 데이터 저장
    const [moduleData, setModuleData] = useState<Record<string, string>>({});

    // 웹소켓 연결 설정
    // useEffect(() => {
    //     // 라즈베리파이 웹소켓 서버 주소 (실제 IP로 변경 필요)
    //     const WEBSOCKET_SERVER = "ws://localhost:8765";
    //     if (!isConnected) {
    //         // 웹소켓 연결 함수
    //         const connectWebSocket = () => {
    //             const ws = new WebSocket(WEBSOCKET_SERVER);

    //             ws.onopen = () => {
    //                 console.log("웹소켓 서버에 연결됨");
    //                 setIsConnected(true);

    //                 // 현재 상태 요청
    //                 ws.send(JSON.stringify({ command: "get_status" }));
    //             };

    //             ws.onmessage = (event) => {
    //                 try {
    //                     const data: WebSocketMessage = JSON.parse(event.data);
    //                     handleWebSocketMessage(data);
    //                 } catch (error) {
    //                     console.error("웹소켓 메시지 처리 중 오류:", error);
    //                 }
    //             };

    //             ws.onerror = (error) => {
    //                 console.error("웹소켓 오류:", error);
    //             };

    //             ws.onclose = () => {
    //                 console.log("웹소켓 연결 종료. 재연결 예정...");
    //                 setIsConnected(false);

    //                 // 5초 후 재연결 시도
    //                 setTimeout(connectWebSocket, 5000);
    //             };

    //             webSocketRef.current = ws;
    //         };

    //         // 초기 연결
    //         connectWebSocket();

    //         // 컴포넌트 언마운트 시 연결 종료
    //         return () => {
    //             if (webSocketRef.current) {
    //                 webSocketRef.current.close();
    //             }
    //         };
    //     }
    // }, []);

    // 웹소켓 메시지 처리 함수
    const handleWebSocketMessage = (data: WebSocketMessage) => {
        console.log("수신된 메시지:", data);

        // type 확인 후 해당하는 모듈 매핑
        const moduleType = data.type;
        const moduleName = moduleTypeMapping[moduleType];

        if (!moduleName) {
            console.log(`지원되지 않는 모듈 타입: ${moduleType}`);
            return;
        }

        // ON/OFF 상태에 따라 모듈 활성화/비활성화
        if (data.contents.default === "ON") {
            // 모듈 데이터 업데이트
            setModuleData((prev) => ({
                ...prev,
                [moduleName]: data.contents.data,
            }));

            // 이미 활성화되어 있지 않으면 활성화
            setActiveModules((prev) => {
                if (!prev[moduleName]) {
                    // 활성화 전에 화면 공간 확인 (세로형 모듈인 경우)
                    if (verticalModules.some((m) => m.name === moduleName)) {
                        const newModules = { ...prev, [moduleName]: true };
                        setPrevActiveModules(prev);
                        return newModules;
                    }
                }
                return { ...prev, [moduleName]: true };
            });

            // IoT 모듈 특별 처리 - 데이터에 따라 상태 변경
            if (moduleType === "iot" && data.contents.data) {
                console.log(`IoT 모듈 활성화: ${data.contents.data}`);
                // IoT 특정 데이터 처리 (필요한 경우)
            }

            // youtube 타입인 경우 데이터에 따라 세로/가로형 결정
            if (moduleType === "youtube" && data.contents.data) {
                // 예: 데이터 형식에 따라 세로/가로형 결정 로직
                // 여기서는 간단하게 구현
            }
        } else {
            // 모듈 비활성화
            setActiveModules((prev) => {
                const newModules = { ...prev };
                delete newModules[moduleName];
                return newModules;
            });

            // IoT 모듈 특별 처리 - 비활성화 시
            if (moduleType === "iot") {
                console.log("IoT 모듈 비활성화");
            }
        }
    };

    // 화면 높이 계산 (버튼 높이 포함)
    useEffect(() => {
        const updateAvailableHeight = () => {
            const viewportHeight = window.innerHeight;
            const buttonHeight = buttonContainerRef.current?.offsetHeight || 0;
            const padding = 20;
            setAvailableHeight(viewportHeight - buttonHeight - padding);
        };

        updateAvailableHeight();
        window.addEventListener("resize", updateAvailableHeight);
        return () => window.removeEventListener("resize", updateAvailableHeight);
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
            alert("세로형 모듈이 화면을 초과하여 더 이상 추가할 수 없습니다.");
            setActiveModules(prevActiveModules);
        }
    }, [activeModules, availableHeight, prevActiveModules]);

    const toggleModule = (name: string) => {
        setPrevActiveModules(activeModules);

        // 모듈 토글 시 웹소켓 메시지 전송
        if (webSocketRef.current && webSocketRef.current.readyState === WebSocket.OPEN) {
            // 모듈 이름을 type으로 변환
            let messageType = name;
            for (const [type, moduleName] of Object.entries(moduleTypeMapping)) {
                if (moduleName === name) {
                    messageType = type;
                    break;
                }
            }

            // 웹소켓 메시지 생성
            const message = {
                command: "process_data",
                data: {
                    type: messageType,
                    contents: {
                        default: activeModules[name] ? "OFF" : "ON",
                        data: moduleData[name] || "",
                    },
                },
            };

            // 메시지 전송
            webSocketRef.current.send(JSON.stringify(message));
        }

        // UI 상태 업데이트
        setActiveModules((prev) => ({ ...prev, [name]: !prev[name] }));
    };

    const removeModule = (name: string) => {
        // 모듈 제거 시 웹소켓 메시지 전송
        if (webSocketRef.current && webSocketRef.current.readyState === WebSocket.OPEN) {
            // 모듈 이름을 type으로 변환
            let messageType = name;
            for (const [type, moduleName] of Object.entries(moduleTypeMapping)) {
                if (moduleName === name) {
                    messageType = type;
                    break;
                }
            }

            // 웹소켓 메시지 생성
            const message = {
                command: "process_data",
                data: {
                    type: messageType,
                    contents: {
                        default: "OFF",
                        data: "",
                    },
                },
            };

            // 메시지 전송
            webSocketRef.current.send(JSON.stringify(message));
        }

        // UI 상태 업데이트
        setActiveModules((prev) => {
            const updatedModules = { ...prev };
            delete updatedModules[name];
            return updatedModules;
        });
    };

    return (
        <div className="bg-black text-white font-sans flex flex-col items-center min-h-screen">
            {/* 버튼 UI */}
            <div ref={buttonContainerRef} className="flex flex-wrap gap-2 mb-6">
                {[...verticalModules, ...horizontalModules].map(({ name }) => (
                    <button
                        key={name}
                        onClick={() => toggleModule(name)}
                        className={`px-2 py-2 rounded-md ${
                            activeModules[name] ? "bg-blue-500 text-white" : "bg-blue-100 hover:bg-blue-200"
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
                    className="flex flex-col w-56 items-center overflow-hidden border-white"
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
                {/* 가로 스택 */}
                <div className="flex flex-wrap w-full h-min justify-end items-start">
                    {horizontalModules.map(({ name, component: Component }) =>
                        activeModules[name] ? (
                            <div key={name} className="w-auto">
                                {name === "timer" ? <Timer onExpire={() => removeModule("timer")} /> : <Component />}
                            </div>
                        ) : null
                    )}
                </div>
            </div>
        </div>
    );
}
