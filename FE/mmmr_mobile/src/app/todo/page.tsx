"use client";

import AddTodo from "@/components/addTodo";
import EditTodo from "@/components/editTodo";
import API_ROUTES from "@/config/apiRoutes";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { AiOutlineEdit } from "react-icons/ai";
import { HiOutlineTrash } from "react-icons/hi";

interface Todo {
    id: number;
    content: string;
    isDone: boolean;
}

export default function Todo() {
    const router = useRouter();
    const [showAddTodoModal, setShowAddTodoModal] = useState(false); // Todo 추가 모달 상태
    const [showEditTodoModal, setShowEditTodoModal] = useState(false); // Todo 추가 모달 상태
    const [todos, setTodos] = useState<Todo[]>([]);
    const [selectedTodo, setSelectedTodo] = useState<Todo>(todos[0]);

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/login");
            return null;
        }

        try {
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`
                },
                body: JSON.stringify({ token: accessToken })
            });

            if (!validateResponse.ok && refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    },
                    body: JSON.stringify({ token: refreshToken })
                });

                const refreshData = await refreshResponse.json();
                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    localStorage.setItem("accessToken", accessToken!);
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/login");
                    return null;
                }
            }

            return accessToken;
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const fetchTodos = async () => {
        const accessToken = await getTokens();
        const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
        const profileId = profile?.id;

        if (!accessToken || !profileId) return;

        try {
            const response = await fetch(API_ROUTES.todos.listByProfile(profileId), {
                method: "GET",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`
                }
            });

            if (response.ok) {
                const data = await response.json();
                setTodos(data.data || []);
            } else {
                console.error("투두 목록 조회 실패");
            }
        } catch (error) {
            console.error("투두 목록 요청 중 오류:", error);
        }
    };

    const handleDeleteTodo = async (todoId: number) => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.todos.delete(todoId), {
                method: "DELETE",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`
                }
            });

            if (response.ok) {
                alert("할일이 삭제되었습니다.");
                fetchTodos(); // 또는 fetchTodos()
            } else {
                const errorData = await response.json();
                console.error("삭제 실패:", errorData);
                alert("삭제에 실패했습니다.");
            }
        } catch (error) {
            console.error("삭제 중 오류:", error);
            alert("네트워크 오류가 발생했습니다.");
        }
    };

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken"); // 'token' -> 'accessToken'으로 수정
            if (token) {
                const profile = localStorage.getItem("currentProfile"); // 'profile' -> 'currentProfile'으로 수정
                if (!profile) {
                    router.push("/profile"); // 프로필이 없으면 프로필 페이지로 리다이렉트
                }
            } else {
                router.push("/login"); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
        fetchTodos();
    }, [router]);

    useEffect(() => {
        fetchTodos();
    }, [showAddTodoModal, showEditTodoModal]); // 새로 추가 후 모달 닫힐 때마다 다시 불러옴

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
                    <div className="flex flex-col space-y-2">
                        {todos.map((todo) => (
                            <div
                                key={todo.id}
                                className={`flex items-center justify-between px-4 py-2 text-gray-600  rounded-full ${
                                    todo.isDone ? "line-through bg-gray-100" : "bg-blue-100"
                                }`}
                            >
                                <span className="text-md">• {todo.content}</span>
                                {todo.isDone ? (
                                    <button className="text-lg" onClick={() => handleDeleteTodo(todo.id)}>
                                        <HiOutlineTrash />
                                    </button>
                                ) : (
                                    <button
                                        className="text-lg"
                                        onClick={() => {
                                            setSelectedTodo(todo);
                                            setShowEditTodoModal(true);
                                        }}
                                    >
                                        <AiOutlineEdit />
                                    </button>
                                )}
                            </div>
                        ))}
                    </div>
                </div>
            </div>
            {showAddTodoModal && <AddTodo onClose={() => setShowAddTodoModal(false)} />}
            {showEditTodoModal && (
                <EditTodo
                    onClose={() => setShowEditTodoModal(false)}
                    todoId={selectedTodo.id}
                    initialContent={selectedTodo.content}
                />
            )}
        </div>
    );
}
