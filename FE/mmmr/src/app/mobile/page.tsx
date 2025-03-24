'use client';

import { useState, useEffect } from 'react';
import LoginPage from '@/components/mobile/login';
import { useRouter } from 'next/navigation';

export default function Page() {
    const [isAuthenticated, setIsAuthenticated] = useState(false);
    const router = useRouter();

    useEffect(() => {
        if (typeof window !== 'undefined') {
            const token = localStorage.getItem('token');
            if (token) {
                setIsAuthenticated(true);
                router.push('/mobile/home'); // 로그인되어 있으면 홈 페이지로 리다이렉트
            } else {
                setIsAuthenticated(false);
                router.push('/mobile/login'); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
    }, [router]);

    return null;
}
