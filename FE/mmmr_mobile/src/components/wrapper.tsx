"use client";

import { useState, useEffect } from "react";
import Header from "@/components/header";
import Footer from "@/components/footer";

export default function Wrapper({ children }: { children: React.ReactNode }) {
    const [isAuthenticated, setIsAuthenticated] = useState(false);

    useEffect(() => {
        const token = localStorage.getItem("accessToken");
        setIsAuthenticated(!!token);
    }, []);

    return (
        <div className="flex justify-center">
            <div className="font-sans w-full max-w-[500px] min-h-screen bg-gray-100 flex flex-col">
                <Header />
                <div
                    className="flex-1 overflow-y-auto px-4 pt-2 
                        pb-[calc(4rem+env(safe-area-inset-bottom))]"
                >
                    {children}
                </div>
                {isAuthenticated && (
                    <div className="fixed bottom-0 left-1/2 -translate-x-1/2 w-full max-w-[500px] z-50">
                        <Footer />
                    </div>
                )}
            </div>
        </div>
    );
}
