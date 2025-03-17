/** @type {import('tailwindcss').Config} */
module.exports = {
    content: ['./src/**/*.{js,ts,jsx,tsx,mdx}'],
    theme: {
        extend: {
            fontFamily: {
                sans: ['Noto Sans KR', 'sans-serif'], // 기본 폰트를 Noto Sans Korean으로 변경
            },
        },
    },
    plugins: [],
};
