"use client";

import { useState, useEffect } from "react";
import Header from "@/components/header";
import Footer from "@/components/footer";

export default function Wrapper({ children }: { children: React.ReactNode }) {
    const [isAuthenticated, setIsAuthenticated] = useState(false);
    const [isMounted, setIsMounted] = useState(false);

    useEffect(() => {
        const token = localStorage.getItem("accessToken");
        setIsAuthenticated(!!token);
        setIsMounted(true); // hydration 이후 mount 표시
    }, []);

    return (
        <div className="flex justify-center">
            <div className="font-sans w-full max-w-[500px] h-[100dvh] min-h-screen  bg-gray-100 flex flex-col">
                <Header />
                <div
                    className={`flex-1 overflow-y-auto px-4 pt-2 ${
                        isAuthenticated ? "pb-[calc(4rem+env(safe-area-inset-bottom))]" : ""
                    }`}
                >
                    {children}
                </div>
                {isMounted && isAuthenticated && (
                    <div className="fixed bottom-0 left-1/2 -translate-x-1/2 w-full max-w-[500px] z-50">
                        <Footer />
                    </div>
                )}
            </div>
        </div>
    );
}
