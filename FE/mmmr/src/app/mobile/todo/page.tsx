"use client";

import AddTodo from "@/components/mobile/addTodo";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";

export default function Todo() {
    const router = useRouter();
    const [showAddTodoModal, setShowAddTodoModal] = useState(false); // Todo 추가 모달 상태

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
                <div className="flex flex-col h-full space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                    <div className="flex justify-between items-center w-full border-b-2 border-blue-300">
                        <h1 className="pl-2 pb-1 text-xl text-blue-300 font-bold">Todo List</h1>
                        <button
                            className="bg-blue-300 rounded-xl text-white px-2 font-bold"
                            onClick={() => setShowAddTodoModal(true)}
                        >
                            +
                        </button>
                    </div>
                </div>
            </div>
            {showAddTodoModal && <AddTodo onClose={() => setShowAddTodoModal(false)} />}
        </div>
    );
}
