"use client";
import { useState } from "react";
import { useRouter } from "next/navigation";
import { ImCross } from "react-icons/im";
import { getTokens } from "@/config/getToken";

interface AddTodoProps {
    onClose: () => void;
}

export default function AddTodo({ onClose }: AddTodoProps) {
    const router = useRouter();
    const [todo, setTodo] = useState("");

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-sm rounded-lg p-6 relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-4">할일 추가</h2>
                <div>
                    <label className="block text-sm text-gray-500 mb-1">내용</label>

                    <div className="mb-4 relative">
                        <input
                            type={"text"}
                            value={todo}
                            onChange={(e) => setTodo(e.target.value)}
                            className="w-full p-2 pb-10 border rounded-md"
                            placeholder="할일을 입력하세요"
                        />
                    </div>
                </div>
                <div className="flex justify-center mt-4">
                    <button className="bg-blue-300 text-white py-2 px-4 rounded-md w-1/3">추가</button>
                    <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                        <ImCross />
                    </button>
                </div>
            </div>
        </div>
    );
}
