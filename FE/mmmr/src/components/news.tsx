'use client';

import { useEffect, useState } from 'react';

export default function News() {
    const [news, setNews] = useState<string[]>([]);
    const [loading, setLoading] = useState<boolean>(true);

    useEffect(() => {
        const fetchNews = async () => {
            try {
                const response = await fetch('http://70.12.246.168:8088/api/news');
                const data = await response.json();
                const titles = data.slice(0, 5).map((item: { title: string }) => item.title); // 상위 5개 뉴스 제목만 추출
                setNews(titles);
            } catch (error) {
                console.error('Failed to fetch news:', error);
            } finally {
                setLoading(false);
            }
        };

        fetchNews();
    }, []);

    const truncateText = (text: string, maxLength: number) => {
        if (text.length <= maxLength) return text;

        const words = text.split(' '); // 공백 기준으로 단어 배열 생성
        let truncated = '';

        for (const word of words) {
            if ((truncated + word).length > maxLength) break; // 최대 길이 초과 시 종료
            truncated += (truncated ? ' ' : '') + word;
        }

        return truncated + '...'; // 단어 단위로 자르고 "..." 추가
    };

    return (
        <div className="font-sans py-3 px-5 w-auto h-44 shadow-md">
            <div>
                <h2 className="text-lg font-semibold">Top5 News</h2>
            </div>
            <div>
                {loading ? (
                    <p>Loading...</p>
                ) : (
                    news.map((item, index) => (
                        <div key={index} className="text-sm flex gap-2 items-center">
                            <p className="font-bold text-base">{index + 1}</p>
                            {item.length > 20 ? <p>{truncateText(item, 20)}</p> : <p>{item}</p>}
                        </div>
                    ))
                )}
            </div>
        </div>
    );
}
