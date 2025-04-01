import { useEffect, useState } from "react";
import API_ROUTES from "@/config/apiRoutes";
import { getToken } from "@/config/getToken";

interface TodoItem {
    id: number;
    content: string;
    isDone: boolean;
}

export default function Todo() {
    const [todoItems, setTodoItems] = useState<TodoItem[]>([]);

    useEffect(() => {
        const fetchTodos = async () => {
            try {
                const accessToken = await getToken();
                const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
                const profileId = profile?.id;
                const response = await fetch(API_ROUTES.todos.listByStatus(profileId), {
                    method: "GET",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`,
                    },
                });

                if (response.ok) {
                    const data = await response.json();
                    setTodoItems(data.data.incompleteTodos);
                } else {
                    console.error("API 응답 오류:", response.status);
                }
            } catch (error) {
                console.error("영상 불러오기 실패:", error);
            }
        };

        fetchTodos();
    }, []); // ✅ 키워드가 변경될 경우 재요청

    return (
        <div className="py-3 px-5 h-auto w-52">
            <h2 className="text-lg font-semibold border-b-2 border-white">Todo List</h2>
            <ul className="pt-2 list-disc list-outside text-sm pl-3">
                {todoItems.length > 0 ? (
                    todoItems.map((item) => (
                        <li key={item.id} className="leading-relaxed break-keep">
                            {item.content}
                        </li>
                    ))
                ) : (
                    <p>할 일이 없습니다</p>
                )}
            </ul>
        </div>
    );
}
