"use client";
import { useRouter } from "next/navigation";
import { useEffect } from "react";

export default function Trans() {
    const router = useRouter();

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken"); // 'token' -> 'accessToken'으로 수정
            if (token) {
                const profile = localStorage.getItem("currentProfile"); // 'profile' -> 'currentProfile'으로 수정
                if (!profile) {
                    router.push("/mobile/profile"); // 프로필이 없으면 프로필 페이지로 리다이렉트
                }
            } else {
                router.push("/mobile/login"); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
    }, []);

    return (
        <div className="flex-1 w-full flex h-full items-center justify-center relative">
            <div className="h-full w-full flex flex-col items-center justify-center p-6">
                <div className="flex flex-col h-full items-center space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                    <h1 className="text-xl text-blue-300 font-bold">Transportation</h1>
                </div>
            </div>
        </div>
    );
}
