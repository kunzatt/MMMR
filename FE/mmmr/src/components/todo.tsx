export default function Todo() {
    const todoItems = [
        '할일 1번 어쩌고 저쩌고',
        '삼성 서류 내기',
        '할일 2번은 또 뭘하고 저걸 하고',
        '집가는길에 이마트 들리기',
        '빨래 돌리고 빨래 널고 빨래 개기',
    ];

    return (
        <div className="font-sans py-3 px-5 h-auto w-48 shadow-md">
            <h2 className="text-lg font-semibold border-b-2 border-black">Todo List</h2>
            <ul className="pt-2 list-disc list-outside text-sm pl-3">
                {todoItems.map((item, index) => (
                    <li key={index} className="leading-relaxed break-keep">
                        {item}
                    </li>
                ))}
            </ul>
        </div>
    );
}
