import { FaBus, FaSubway } from 'react-icons/fa';

const rawData = [
    {
        type: 'bus',
        station: '역삼역7번출구.GS타워',
        direction: '어쩌고 저쩌고 방향',
        number: '463번',
        time: '5분 뒤 도착',
    },
    { type: 'bus', station: '차병원', direction: '어쩌고 저쩌고 방향', number: '4211번', time: '3분 뒤 도착' },
    { type: 'subway', station: '역삼역', direction: '강남행', number: '2호선', time: '2분 뒤 도착' },
];

export default function Transportation() {
    return (
        <div className="py-1 px-5 h-auto w-52">
            {rawData.map((item, index) => (
                <div
                    key={index}
                    className={`flex items-center py-2 ${index !== rawData.length - 1 ? 'border-b border-black' : ''}`}
                >
                    {/* 아이콘 */}
                    <div className="text-2xl mr-3">{item.type === 'bus' ? <FaBus /> : <FaSubway />}</div>

                    {/* 정보 */}
                    <div className="text-left flex-grow">
                        <p className="text-sm font-bold">
                            {item.station.length < 8 ? item.station : item.station.slice(0, 8) + '...'}
                        </p>
                        <p className="text-xs text-gray-600">
                            {item.direction.length <= 10 ? item.direction : item.direction.slice(0, 10) + '...'}
                        </p>
                        <div className="text-sm flex justify-between font-bold mt-1">
                            <span>{item.number}</span>
                            <span>{item.time}</span>
                        </div>
                    </div>
                </div>
            ))}
        </div>
    );
}
