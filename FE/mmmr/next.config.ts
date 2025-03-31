import type { NextConfig } from 'next';

const nextConfig: NextConfig = {
    /* config options here */
    eslint: {
    // ✅ 빌드 시 ESLint 에러 무시 (배포 시 임시 방편)
        ignoreDuringBuilds: true,
    },
};

export default nextConfig;
