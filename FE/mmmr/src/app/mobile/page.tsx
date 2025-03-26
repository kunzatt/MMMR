"use client";

import { useState, useEffect } from "react";
import { useRouter } from "next/navigation";

export default function Page() {
    const [isAuthenticated, setIsAuthenticated] = useState(false);
    const router = useRouter();

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken"); // 'token' -> 'accessToken'으로 수정
            if (token) {
                setIsAuthenticated(true);
                router.push("/mobile/profile"); // 로그인되어 있으면 프로필필 페이지로 리다이렉트
            } else {
                setIsAuthenticated(false);
                router.push("/mobile/login"); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
    }, [router]);

    return null;
}
