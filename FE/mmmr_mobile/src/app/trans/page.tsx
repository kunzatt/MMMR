"use client";
import AddTrans from "@/components/addTrans";
import API_ROUTES from "@/config/apiRoutes";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { HiOutlineTrash } from "react-icons/hi2";

interface Transportation {
    id: number;
    type: "BUS" | "METRO";
    route: string;
    station: string;
    direction?: string;
    line?: number;
    routeId: string;
    stationId: string;
}

export default function Trans() {
    const router = useRouter();
    const [showAddTransModal, setShowAddTransModal] = useState(false);
    const [transportations, setTransportations] = useState<Transportation[]>([]);

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

    const fetchTransports = async () => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
        if (!profile?.id) {
            router.push("/profile");
            return;
        }

        try {
            const res = await fetch(API_ROUTES.trans.listByProfile(profile.id), {
                headers: {
                    Authorization: `Bearer ${accessToken}`
                }
            });

            if (res.ok) {
                const data = await res.json();
                const all = [...data.data.buses, ...data.data.metros];
                setTransportations(all);
            } else {
                console.error("교통 정보 불러오기 실패");
            }
        } catch (err) {
            console.error("네트워크 오류:", err);
        }
    };

    const deleteTransportation = async (id: number, type: string) => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const res = await fetch(`${API_ROUTES.trans.delete(id)}?type=${type}`, {
                method: "DELETE",
                headers: { Authorization: `Bearer ${accessToken}` }
            });

            if (res.ok) {
                setTransportations((prev) => prev.filter((t) => t.id !== id));
            } else {
                const err = await res.json();
                console.error("삭제 실패:", err);
            }
        } catch (err) {
            console.error("삭제 오류:", err);
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
        fetchTransports();
    }, [router]);

    useEffect(() => {
        fetchTransports();
        console.log(transportations);
    }, [showAddTransModal]);

    return (
        <div className="flex-1 w-full flex h-full items-center justify-center relative">
            <div className="h-full w-full flex flex-col items-center justify-center p-6">
                <div className="flex flex-col h-full p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                    <div className="flex justify-between items-center w-full border-b-2 border-blue-300 mb-4">
                        <h1 className="pl-2 pb-1 text-xl text-blue-300 font-bold">Transportation</h1>
                        <button
                            className="bg-blue-300 rounded-xl text-white px-2 font-bold"
                            onClick={() => setShowAddTransModal(true)}
                        >
                            +
                        </button>
                    </div>
                    {/* 리스트 */}
                    {transportations.length === 0 ? (
                        <p className="text-center text-gray-400 mt-6">등록된 교통 정보가 없습니다.</p>
                    ) : (
                        <>
                            {/* 버스 */}
                            <h2 className="text-md text-blue-300 font-bold pl-2">버스</h2>
                            {transportations
                                .filter((item) => item.type === "BUS")
                                .map((bus) => (
                                    <div
                                        key={bus.id}
                                        className="flex items-center justify-between px-4 py-2 text-gray-600 bg-blue-100 rounded-3xl mt-2 mb-2"
                                    >
                                        <div className="text-xl font-bold ">{bus.route}번</div>
                                        <div className="flex gap-4">
                                            <div className="flex flex-col items-end">
                                                <div className="text-md font-semibold">{bus.station}</div>
                                                <div className="text-xs ">{bus.direction} 방향</div>
                                            </div>
                                            <button
                                                onClick={() => deleteTransportation(bus.id, bus.type)}
                                                className="text-gray-500"
                                            >
                                                <HiOutlineTrash size={20} />
                                            </button>
                                        </div>
                                    </div>
                                ))}

                            {/* 지하철 */}
                            <h2 className="text-md text-blue-300 font-bold pl-2 mt-4">지하철</h2>
                            {transportations
                                .filter((item) => item.type === "METRO")
                                .map((metro) => (
                                    <div
                                        key={metro.id}
                                        className="flex items-center justify-between px-4 py-3 text-gray-600 bg-blue-100 rounded-3xl mt-2 mb-2"
                                    >
                                        <div className="text-xl font-bold ">{metro.station}</div>
                                        <div className="flex gap-4">
                                            <div className="text-md font-semibold">{metro.line}</div>
                                            <button
                                                onClick={() => deleteTransportation(metro.id, metro.type)}
                                                className="text-gray-500"
                                            >
                                                <HiOutlineTrash size={20} />
                                            </button>
                                        </div>
                                    </div>
                                ))}
                        </>
                    )}
                </div>
            </div>
            {showAddTransModal && <AddTrans onClose={() => setShowAddTransModal(false)} />}
        </div>
    );
}
