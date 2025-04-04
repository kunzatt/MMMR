"use client";

import { useEffect, useState } from "react";
import { ImCross } from "react-icons/im";
import QRCode from "react-qr-code";

interface ShowQrProps {
    onClose: () => void;
}

export default function ShowQr({ onClose }: ShowQrProps) {
    const [qrData, setQrData] = useState("");

    useEffect(() => {
        const email = localStorage.getItem("email") || "";
        const password = localStorage.getItem("password") || "";

        const payload = {
            email,
            password
        };

        setQrData(JSON.stringify(payload));
    }, []);

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-sm rounded-3xl p-6 relative text-center">
                <h2 className="text-xl text-blue-300 font-bold mb-4">로그인 QR</h2>

                {qrData ? (
                    <div className="bg-white p-4 rounded flex justify-center">
                        <QRCode value={qrData} size={256} />
                    </div>
                ) : (
                    <p className="text-gray-500">로딩 중...</p>
                )}

                <button className="absolute top-4 right-4 text-gray-400 hover:text-gray-600" onClick={onClose}>
                    <ImCross size={16} />
                </button>
            </div>
        </div>
    );
}
