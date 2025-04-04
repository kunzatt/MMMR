"use client";

import { useState } from "react";

export default function Homecam() {
    const [isLoaded, setIsLoaded] = useState(false);

    return (
        <div className="py-3 px-5 w-auto h-44">
            <h2 className="text-lg font-semibold mb-2">📸 실시간 영상 스트리밍</h2>
            <div className="w-64 h-36 bg-black rounded-md overflow-hidden flex items-center justify-center">
                {!isLoaded && <p className="text-white text-sm">로딩 중...</p>}
                <img
                    src="http://70.12.246.50:5000/video_feed"
                    alt="실시간 스트리밍"
                    className="w-full h-full object-cover"
                    onLoad={() => setIsLoaded(true)}
                    onError={() => {
                        setIsLoaded(true);
                        console.error("스트리밍 로드 실패");
                    }}
                />
            </div>
        </div>
    );
}
