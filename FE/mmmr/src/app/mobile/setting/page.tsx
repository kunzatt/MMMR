"use client";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import Footer from "@/components/mobile/footer";
import ChangePassword from "@/components/mobile/changePassword";
import DeleteAccount from "@/components/mobile/deleteAccount";

export default function Setting() {
    const router = useRouter();
    const [showChangePasswordModal, setShowChangePasswordModal] = useState(false);
    const [showDeleteAccountModal, setShowDeleteAccountModal] = useState(false);

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken");
            if (token) {
                const profile = localStorage.getItem("currentProfile");
                if (!profile) {
                    router.push("/mobile/profile");
                }
            } else {
                router.push("/mobile/login");
            }
        }
    }, []);

    return (
        <div className="flex-1 w-full flex h-full items-center justify-center relative">
            <div className="h-full w-full flex flex-col items-center justify-center p-6">
                <div className="flex flex-col h-full space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                    <h1 className="pl-2 pb-1 text-xl text-blue-300 font-bold border-b-2 border-blue-300">Setting</h1>
                    <button
                        className="w-full bg-blue-300 text-white py-2 px-4 rounded-full"
                        onClick={() => setShowChangePasswordModal(true)}
                    >
                        비밀번호 변경
                    </button>
                    <button
                        className="w-full bg-blue-300 text-white py-2 px-4 rounded-full"
                        onClick={() => router.push("/mobile/profile")}
                    >
                        프로필 관리
                    </button>
                    <button
                        className="w-full bg-blue-300 text-white py-2 px-4 rounded-full"
                        onClick={() => setShowDeleteAccountModal(true)}
                    >
                        회원 탈퇴
                    </button>
                </div>
            </div>
            {showChangePasswordModal && <ChangePassword onClose={() => setShowChangePasswordModal(false)} />}
            {showDeleteAccountModal && <DeleteAccount onClose={() => setShowDeleteAccountModal(false)} />}
        </div>
    );
}
