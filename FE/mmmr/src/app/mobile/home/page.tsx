"use client";
import { useRouter } from "next/navigation";
import { useEffect } from "react";

export default function Home() {
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

        if (sessionStorage.getItem("hasReloaded") === "true") {
            sessionStorage.removeItem("hasReloaded"); // 플래그 삭제 (다시 새로고침 되지 않게 하기)
            window.location.reload(); // 새로고침 실행
        }
    }, []);

    return (
        <div className="flex-1 w-full flex items-center justify-center">
            <h1 className="text-2xl">홈 페이지 - 로그인 성공!</h1>
        </div>
    );
}
