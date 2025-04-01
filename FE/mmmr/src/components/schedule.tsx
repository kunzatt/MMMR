export default function Schedule() {
    const scheduleData = [
        {
            date: "March 10",
            tasks: ["피그마 만들기", "DB 구조 설계하기"],
        },
        {
            date: "March 12",
            tasks: ["OPIC 시험 준비"],
        },
        {
            date: "March 17",
            tasks: ["삼성전자 DX 서류 마감"],
        },
    ];
    return (
        <div className="py-3 px-5 h-auto w-52">
            {scheduleData.map((entry, index) => (
                <div key={index} className={`${index === scheduleData.length - 1 ? "" : "mb-4"}`}>
                    <h3 className="text-lg font-semibold border-b-2 border-white">{entry.date}</h3>
                    <div className="pt-2 flex flex-col gap-2">
                        {entry.tasks.map((task, taskIndex) => (
                            <div
                                key={taskIndex}
                                className="px-3 py-1 border border-white rounded-md text-sm break-keep text-center"
                            >
                                {task}
                            </div>
                        ))}
                    </div>
                </div>
            ))}
        </div>
    );
}
