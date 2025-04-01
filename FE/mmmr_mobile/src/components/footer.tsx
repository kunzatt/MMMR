"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { AiFillHome, AiFillCalendar, AiOutlineCheckSquare, AiOutlineCompass, AiFillSetting } from "react-icons/ai";

export default function Footer() {
    const pathname = usePathname(); // 현재 경로를 가져옴

    const menuItems = [
        { name: "Home", path: "/home", icon: <AiFillHome /> },
        { name: "Todo", path: "/todo", icon: <AiOutlineCheckSquare /> },
        { name: "Schedule", path: "/schedule", icon: <AiFillCalendar /> },
        { name: "Trans", path: "/trans", icon: <AiOutlineCompass /> },
        { name: "Setting", path: "/setting", icon: <AiFillSetting /> },
    ];

    return (
        <div>
            {pathname !== "/profile" && (
                <div className="flex justify-between items-center p-0 bg-white shadow-inner">
                    {menuItems.map((item) => {
                        const isActive = pathname === item.path; // 현재 경로와 메뉴 경로 비교

                        return (
                            <Link href={item.path} key={item.name} className="w-full">
                                <div
                                    className={`flex flex-col items-center justify-center py-3 w-full ${
                                        isActive ? "bg-blue-100" : ""
                                    }`}
                                >
                                    <div className={`text-2xl ${isActive ? "text-blue-300" : "text-gray-500"}`}>
                                        {item.icon}
                                    </div>
                                    <span className={`text-sm ${isActive ? "text-blue-300" : "text-gray-500"}`}>
                                        {item.name}
                                    </span>
                                </div>
                            </Link>
                        );
                    })}
                </div>
            )}
        </div>
    );
}
