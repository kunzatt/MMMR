"use client";

import { useState } from "react";

export default function Homecam() {
    const [isLoaded, setIsLoaded] = useState(false);

    return (
        <div className="py-3 px-5 w-auto h-44">
            <div className="w-64 h-36 bg-black rounded-md overflow-hidden flex items-center justify-center">
                {!isLoaded && <p className="w-full text-white text-sm">로딩 중...</p>}
                <img
                    src="https://7f67-211-192-210-16.ngrok-free.app/video_feed"
                    alt="홈캠 로딩 실패"
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
