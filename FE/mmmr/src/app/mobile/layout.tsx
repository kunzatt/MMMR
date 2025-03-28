"use client";

import { useState, useEffect } from "react";
import Header from "@/components/mobile/header";
import Footer from "@/components/mobile/footer";

export default function Layout({ children }: { children: React.ReactNode }) {
    const [isAuthenticated, setIsAuthenticated] = useState(false);

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken");
            setIsAuthenticated(!!token); // 토큰이 있으면 true, 없으면 false
        }
    }, []); // 빈 배열을 의존성으로 지정하여 처음 로드될 때 한 번만 실행

    return (
        <div className="flex justify-center">
            <div className="font-sans w-full max-w-[500px] h-screen bg-gray-100 flex flex-col">
                <Header />
                <div className="flex-1 overflow-auto">{children}</div>
                {isAuthenticated && <Footer />} {/* 로그인 상태일 때 Footer 표시 */}
            </div>
        </div>
    );
}
