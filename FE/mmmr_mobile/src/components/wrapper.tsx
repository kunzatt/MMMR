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
            <div className="font-sans w-full max-w-[500px] h-[100dvh] bg-gray-100 flex flex-col">
                <Header />
                <div className="flex-1 overflow-y-auto min-h-0">{children}</div>
                {isAuthenticated && <Footer />}
            </div>
        </div>
    );
}
