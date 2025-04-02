"use client";
import AddTrans from "@/components/addTrans";
import API_ROUTES from "@/config/apiRoutes";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";

interface Transportation {
    id: number;
    type: "BUS" | "METRO";
    route: string;
    station: string;
    direction?: string;
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
                    router.push("/login");
                    return null;
                }
            } else {
                localStorage.removeItem("accessToken");
                router.push("/login");
                return null;
            }
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
                    Authorization: `Bearer ${accessToken}`,
                },
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
                                        className="flex items-center justify-between px-4 py-2 text-gray-600 bg-blue-100 rounded-2xl mt-2 mb-2"
                                    >
                                        <div className="text-xl font-bold ">{bus.route}번</div>
                                        <div className="flex">
                                            <div className="flex flex-col items-end">
                                                <div className="text-md font-semibold">{bus.station}</div>
                                                <div className="text-xs ">{bus.direction} 방향</div>
                                            </div>
                                        </div>
                                    </div>
                                ))}

                            {/* 지하철 */}
                            <h2 className="text-md text-blue-300 font-bold pl-2">지하철</h2>
                            {transportations
                                .filter((item) => item.type === "METRO")
                                .map((metro) => (
                                    <div
                                        key={metro.id}
                                        className="items-center justify-between px-4 py-2 text-gray-600 bg-blue-100 rounded-2xl mt-2 mb-2"
                                    >
                                        <div className="text-lg font-bold ">{metro.route}</div>
                                        <div className="text-sm ">{metro.direction}행</div>
                                        <div className="text-md ">{metro.station}</div>
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
