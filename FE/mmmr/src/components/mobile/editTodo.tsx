"use client";
import { useState } from "react";
import { useRouter } from "next/navigation";
import { ImCross } from "react-icons/im";
import API_ROUTES from "@/config/apiRoutes";

interface EditTodoProps {
    onClose: () => void;
    todoId: number;
    initialContent: string;
}

export default function EditTodo({ onClose, todoId, initialContent }: EditTodoProps) {
    const router = useRouter();
    const [todo, setTodo] = useState(initialContent);

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/mobile/login");
            return null;
        }

        try {
            // 1. 액세스 토큰 유효성 확인
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${refreshToken}`,
                },
                body: JSON.stringify({ token: accessToken }),
            });

            let validateData = null;
            if (validateResponse.ok) {
                const contentType = validateResponse.headers.get("Content-Type");
                if (contentType && contentType.includes("application/json")) {
                    validateData = await validateResponse.json();
                }
            }

            // 2. 토큰이 만료되었거나 유효하지 않은 경우, 리프레시 토큰으로 새 액세스 토큰 발급
            if (refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${refreshToken}`,
                    },
                    body: JSON.stringify({ token: refreshToken }),
                });

                let refreshData = null;
                if (refreshResponse.ok) {
                    const contentType = refreshResponse.headers.get("Content-Type");
                    if (contentType && contentType.includes("application/json")) {
                        refreshData = await refreshResponse.json();
                    }
                }

                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    if (accessToken) {
                        localStorage.setItem("accessToken", accessToken);
                    }
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/mobile/login");
                    return null;
                }
            } else {
                localStorage.removeItem("accessToken");
                router.push("/mobile/login");
                return null;
            }
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const handleUpdateTodo = async () => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.todos.update(todoId), {
                method: "PUT",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ content: todo }),
            });

            if (response.ok) {
                alert("할일이 성공적으로 수정되었습니다.");
                onClose();
                router.refresh(); // 리스트 새로고침
            } else {
                const errorData = await response.json();
                console.error("에러 응답:", errorData);
                alert("할일 수정에 실패했습니다.");
            }
        } catch (error) {
            console.error("할일 수정 중 오류:", error);
            alert("네트워크 오류가 발생했습니다.");
        }
    };

    const handleDeleteTodo = async () => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.todos.delete(todoId), {
                method: "DELETE",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                alert("할일이 성공적으로 삭제되었습니다.");
                onClose();
                router.refresh(); // 리스트 새로고침
            } else {
                const errorData = await response.json();
                console.error("에러 응답:", errorData);
                alert("할일 삭제에 실패했습니다.");
            }
        } catch (error) {
            console.error("할일 삭제 중 오류:", error);
            alert("네트워크 오류가 발생했습니다.");
        }
    };

    const handleDoneTodo = async () => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.todos.toggle(todoId), {
                method: "PUT",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                alert("할일이 완료되었습니다.");
                onClose();
                router.refresh(); // 완료 후 리스트 새로고침
            } else {
                const errorData = await response.json();
                console.error("에러 응답:", errorData);
                alert("할일 완료 처리에 실패했습니다.");
            }
        } catch (error) {
            console.error("완료 처리 중 오류:", error);
            alert("네트워크 오류가 발생했습니다.");
        }
    };

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
                            placeholder="할일을 수정하세요"
                        />
                    </div>
                </div>
                <div className="flex justify-center mt-4 gap-5">
                    <button className="bg-blue-300 text-white py-2 px-4 rounded-md w-1/3" onClick={handleUpdateTodo}>
                        수정
                    </button>
                    <button className="bg-green-300 text-white py-2 px-4 rounded-md w-1/3" onClick={handleDoneTodo}>
                        완료
                    </button>
                    <button className="bg-red-300 text-white py-2 px-4 rounded-md w-1/3" onClick={handleDeleteTodo}>
                        삭제
                    </button>
                    <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                        <ImCross />
                    </button>
                </div>
            </div>
        </div>
    );
}
