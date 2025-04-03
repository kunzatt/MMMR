"use client";

import { useEffect, useState } from "react";
import API_ROUTES from "@/config/apiRoutes";
import { getToken } from "@/config/getToken";

interface YoutubeProps {
    keyword: string;
}

export default function Youtube({ keyword }: YoutubeProps) {
    const [videoId, setVideoId] = useState<string | null>(null);

    useEffect(() => {
        const fetchYoutubeVideos = async () => {
            try {
                const accessToken = await getToken();

                const response = await fetch(`${API_ROUTES.youtube}?keyword=${encodeURIComponent(keyword)}`, {
                    method: "GET",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    }
                });

                if (response.ok) {
                    const data = await response.json();
                    const firstVideo = data.data?.[0]?.videoId;
                    if (firstVideo) {
                        setVideoId(firstVideo);
                    } else {
                        console.warn("영상이 없습니다.");
                    }
                } else {
                    console.error("API 응답 오류:", response.status);
                }
            } catch (error) {
                console.error("영상 불러오기 실패:", error);
            }
        };

        fetchYoutubeVideos();
    }, [keyword]); // ✅ 키워드가 변경될 경우 재요청

    return (
        <div className="py-3 px-5 w-auto h-44 text-center">
            <div className="relative w-full h-full flex items-center justify-center">
                <div className="w-[100%] h-[100%] max-w-[560px] aspect-[16/9] overflow-hidden">
                    <iframe
                        className="w-full h-full rounded-md object-cover"
                        src={`https://www.youtube.com/embed/${videoId}?autoplay=1&controls=0&modestbranding=1&rel=0&showinfo=0`}
                        title="YouTube video player"
                        allow="autoplay; encrypted-media; picture-in-picture"
                        allowFullScreen
                    ></iframe>
                </div>
            </div>
        </div>
    );
}
