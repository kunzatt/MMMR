'use client';

import { useState } from 'react';

export default function Hyoutube() {
    const [url, setUrl] = useState('');
    const [videoId, setVideoId] = useState('');

    // 유튜브 URL에서 videoId 추출하는 함수
    const extractVideoId = (url: string) => {
        const regex =
            /(?:https?:\/\/)?(?:www\.)?(?:youtube\.com\/(?:[^\/\n\s]+\/\S+\/|(?:v|e(?:mbed)?)\/|.*[?&]v=)|youtu\.be\/)([^"&?\/\s]{11})/;
        const match = url.match(regex);
        return match ? match[1] : '';
    };

    const handleSubmit = () => {
        const id = extractVideoId(url);
        setVideoId(id);
    };

    return (
        <div className="py-3 px-5 w-auto h-44 text-center">
            {!videoId && (
                <div>
                    <h2 className="text-lg font-semibold mb-3">YouTube Player</h2>
                    <input
                        type="text"
                        placeholder="Enter YouTube URL"
                        value={url}
                        onChange={(e) => setUrl(e.target.value)}
                        className="w-full p-2 border rounded-md"
                    />
                    <button onClick={handleSubmit} className="mt-2 px-3 py-1 bg-blue-600 text-white rounded-md">
                        Play
                    </button>
                </div>
            )}
            {videoId && (
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
            )}
        </div>
    );
}
