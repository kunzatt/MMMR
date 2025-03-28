"use client";

import { useState, useEffect } from "react";
import { useRouter } from "next/navigation";
import { AiOutlineEdit, AiOutlinePlus } from "react-icons/ai";
import { API_ROUTES } from "@/config/apiRoutes";

export default function ProfilePage() {
    interface Profile {
        id: string;
        nickname: string;
        callSign: string;
    }

    const [profiles, setProfiles] = useState<Profile[]>([]);
    const [showAddModal, setShowAddModal] = useState(false);
    const [showEditModal, setShowEditModal] = useState(false);
    const [currentProfile, setCurrentProfile] = useState<Profile | null>(null);
    const [nickname, setNickname] = useState("");
    const [callSign, setCallSign] = useState("");
    const [availableCallSigns, setAvailableCallSigns] = useState<string[]>([]);
    const [editCallSigns, setEditCallSigns] = useState<string[]>([]);
    const router = useRouter();

    const getTokens = () => {
        const accessToken = localStorage.getItem("accessToken");

        if (!accessToken) {
            alert("로그인이 필요합니다.");
            router.push("/mobile/login");
            return;
        }
        return accessToken;
    };

    const fetchProfiles = async () => {
        const accessToken = getTokens();
        if (!accessToken) return;
        try {
            const response = await fetch(API_ROUTES.profiles.list, {
                method: "GET",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                const data = await response.json();
                setProfiles(data.data || []);
            } else {
                alert("프로필을 불러오는 데 실패했습니다.");
            }
        } catch (error) {
            console.error("프로필 불러오기 오류:", error);
        }
    };

    const fetchCallSigns = async () => {
        const accessToken = getTokens();
        if (!accessToken) return;
        try {
            const response = await fetch(API_ROUTES.profiles.availableCallsigns, {
                method: "GET",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
            });
            const data = await response.json();
            if (response.ok) {
                setAvailableCallSigns(data.data.map((item: { name: any }) => item.name));
                if (data.data.length > 0) setCallSign(data.data[0].callSign); // 첫 번째 값을 기본값으로 설정
            } else {
                console.error("콜사인 목록 조회 실패:", data.message);
            }
        } catch (error) {
            console.error("콜사인 목록 불러오기 오류:", error);
        }
    };

    useEffect(() => {
        const accessToken = getTokens();
        if (accessToken) {
            fetchProfiles();
            fetchCallSigns();
        }
    }, [router]);

    useEffect(() => {
        if (sessionStorage.getItem("hasReloaded") === "true") {
            sessionStorage.removeItem("hasReloaded"); // 플래그 삭제 (다시 새로고침 되지 않게 하기)
            window.location.reload(); // 새로고침 실행
        }
    }, []);

    useEffect(() => {
        fetchCallSigns();
    }, [showAddModal]);

    const handleAddProfile = async () => {
        const accessToken = getTokens();

        try {
            const response = await fetch(API_ROUTES.profiles.add, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ nickname, callSign }),
            });

            if (response.ok) {
                alert("프로필이 성공적으로 추가되었습니다.");
                setShowAddModal(false);
                setNickname("");
                setCallSign("");
                fetchProfiles();
            } else {
                alert("프로필 추가에 실패했습니다.");
            }
        } catch (error) {
            console.error("프로필 추가 오류:", error);
        }
    };
    const handleEditProfile = async () => {
        const accessToken = getTokens();

        try {
            const response = await fetch(API_ROUTES.profiles.update(currentProfile.id), {
                method: "PUT",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ nickname, callSign }), // callSign 대신 editCallSign 사용
            });

            if (response.ok) {
                alert("프로필이 성공적으로 수정되었습니다.");
                setShowEditModal(false);
                setEditCallSigns([]);
                fetchCallSigns();
                fetchProfiles(); // 수정 후 목록 다시 가져오기
            } else {
                alert("프로필 수정에 실패했습니다.");
            }
        } catch (error) {
            console.error("프로필 수정 오류:", error);
        }
    };

    const handleDeleteProfile = async () => {
        const accessToken = getTokens();

        try {
            const response = await fetch(API_ROUTES.profiles.delete(currentProfile.id), {
                method: "DELETE",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                alert("프로필이 성공적으로 삭제되었습니다.");
                setShowEditModal(false);
                fetchProfiles();
                setNickname("");
                setCallSign("");
            } else {
                alert("프로필 삭제에 실패했습니다.");
            }
        } catch (error) {
            console.error("프로필 삭제 오류:", error);
        }
    };

    return (
        <div className="flex flex-col items-center justify-center h-full w-full">
            <div className="flex flex-wrap gap-6 justify-center p-10 mt-4 pb-36">
                {profiles.map((profile) => (
                    <div
                        key={profile.id}
                        className="relative flex items-center justify-center w-24 h-24 bg-blue-300 rounded-2xl cursor-pointer"
                        onClick={() => {
                            localStorage.setItem("currentProfile", JSON.stringify(profile));
                            router.push("/mobile/home");
                        }}
                    >
                        <span className="text-white text-lg">{profile.nickname}</span>
                        <AiOutlineEdit
                            className="absolute top-1 right-1 text-white"
                            onClick={() => {
                                setCurrentProfile(profile);
                                setNickname(profile.nickname);
                                setEditCallSigns(() => [profile.callSign, ...availableCallSigns]);
                                setShowEditModal(true);
                            }}
                        />
                    </div>
                ))}
                <div
                    onClick={() => setShowAddModal(true)}
                    className="flex items-center justify-center w-24 h-24 bg-blue-300 rounded-2xl cursor-pointer"
                >
                    <AiOutlinePlus className="text-white text-3xl" />
                </div>
            </div>

            {showEditModal && (
                <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
                    <div className="bg-white w-10/12 max-w-sm rounded-lg p-6">
                        <h2 className="text-lg font-medium text-center mb-4">프로필 수정</h2>

                        <div className="mb-4">
                            <label className="block text-sm text-gray-500 mb-1">프로필 명</label>
                            <input
                                className="w-full p-2 border rounded-md"
                                type="text"
                                value={nickname}
                                onChange={(e) => setNickname(e.target.value)}
                            />
                        </div>

                        <div className="mb-4">
                            <label className="block text-sm text-gray-500 mb-1">비서 명</label>
                            <select
                                className="w-full p-2 border rounded-md"
                                value={callSign}
                                onChange={(e) => {
                                    setCallSign(e.target.value);
                                }}
                            >
                                {editCallSigns.map((sign) => (
                                    <option key={sign} value={sign}>
                                        {sign}
                                    </option>
                                ))}
                            </select>
                        </div>

                        <div className="flex justify-between mt-4">
                            <button onClick={handleEditProfile} className="bg-blue-300 text-white py-2 px-4 rounded-md">
                                수정하기
                            </button>
                            <button
                                onClick={handleDeleteProfile}
                                className="bg-red-300 text-white py-2 px-4 rounded-md"
                            >
                                삭제하기
                            </button>
                            <button
                                onClick={() => {
                                    setCurrentProfile(null);
                                    setNickname("");
                                    setCallSign("");
                                    setShowEditModal(false);
                                }}
                                className="bg-gray-300 text-white py-2 px-4 rounded-md"
                            >
                                취소
                            </button>
                        </div>
                    </div>
                </div>
            )}

            {showAddModal && (
                <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
                    <div className="bg-white w-10/12 max-w-sm rounded-lg p-6">
                        <h2 className="text-lg font-medium text-center mb-4">프로필 추가</h2>
                        <div className="mb-4">
                            <label className="block text-sm text-gray-500 mb-1">프로필 명</label>
                            <input
                                className="w-full p-2 border rounded-md"
                                type="text"
                                value={nickname}
                                onChange={(e) => setNickname(e.target.value)}
                            />
                        </div>
                        <div className="mb-4">
                            <label className="block text-sm text-gray-500 mb-1">비서 명</label>
                            <select
                                className="w-full p-2 border rounded-md"
                                value={callSign}
                                onChange={(e) => setCallSign(e.target.value)}
                            >
                                {availableCallSigns.map((sign) => (
                                    <option key={sign} value={sign}>
                                        {sign}
                                    </option>
                                ))}
                            </select>
                        </div>
                        <div className="flex justify-between mt-4">
                            <button onClick={handleAddProfile} className="bg-blue-300 text-white py-2 px-4 rounded-md">
                                추가하기
                            </button>
                            <button
                                onClick={() => {
                                    setNickname("");
                                    setCallSign("");
                                    setShowAddModal(false);
                                }}
                                className="bg-gray-300 text-white py-2 px-4 rounded-md"
                            >
                                취소
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}
