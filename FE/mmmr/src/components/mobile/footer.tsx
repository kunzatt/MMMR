'use client';

import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { AiFillHome, AiFillCalendar, AiOutlineCheckSquare, AiOutlineCompass, AiFillSetting } from 'react-icons/ai';

export default function Footer() {
    const pathname = usePathname(); // 현재 경로를 가져옴

    const menuItems = [
        { name: 'Home', path: '/mobile/home', icon: <AiFillHome /> },
        { name: 'Schedule', path: '/mobile/schedule', icon: <AiFillCalendar /> },
        { name: 'Todo', path: '/mobile/todo', icon: <AiOutlineCheckSquare /> },
        { name: 'Trans', path: '/mobile/trans', icon: <AiOutlineCompass /> },
        { name: 'Setting', path: '/mobile/setting', icon: <AiFillSetting /> },
    ];

    return (
        <div className="flex justify-between items-center p-0 bg-white shadow-inner">
            {menuItems.map((item) => {
                const isActive = pathname === item.path; // 현재 경로와 메뉴 경로 비교

                return (
                    <Link href={item.path} key={item.name} className="w-full">
                        <div
                            className={`flex flex-col items-center justify-center py-3 w-full ${
                                isActive ? 'bg-blue-100' : ''
                            }`}
                        >
                            <div className={`text-2xl ${isActive ? 'text-blue-300' : 'text-gray-500'}`}>
                                {item.icon}
                            </div>
                            <span className={`text-sm ${isActive ? 'text-blue-300' : 'text-gray-500'}`}>
                                {item.name}
                            </span>
                        </div>
                    </Link>
                );
            })}
        </div>
    );
}
