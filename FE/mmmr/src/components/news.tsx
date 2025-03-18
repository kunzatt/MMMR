export default function News() {
    const news: Array<string> = [
        '테슬라 시총 하루만에 190조 증발…트럼프 당선 상승분',
        '[단독] 수원 일가족 4명 사망… “40대 가장, 빌려준',
        '초인종 울려 나갔다가…자녀 앞에서 무차별 폭행 당한',
        '유재석, 현금 200억 주고 산 논현동 땅에 빌딩 올린',
        '"이젠 정말 한계다"…맘스터치 일부 사장님들, 결국',
    ];
    return (
        <div className="font-sans py-3 px-5 w-auto h-44 rounded-lg shadow-md">
            <div>
                <h2 className="text-lg font-semibold">Top5 News</h2>
            </div>
            <div>
                {news.map((item, index) => (
                    <div key={index} className="text-sm flex gap-2 items-center">
                        <p className="font-bold text-base">{index + 1}</p>
                        {item.length > 20 && <p>{item.slice(0, 20)}...</p>}
                        {item.length <= 20 && <p>{item}</p>}
                    </div>
                ))}
            </div>
        </div>
    );
}
