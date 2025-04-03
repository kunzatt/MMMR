import API_ROUTES from "@/config/apiRoutes";
import { getToken } from "@/config/getToken";
import { useEffect, useState } from "react";
import { FaBus, FaSubway } from "react-icons/fa";

interface Transportation {
    id: number;
    type: "BUS" | "METRO";
    number: string;
    station: string;
    direction: string;
    information: string;
}

interface Metro {
    id: number;
    number: string;
    station: string;
    information: {
        dir1: string;
        info1: string;
        dir2: string;
        info2: string;
    };
}

export default function Transportation() {
    const [metros, setMetros] = useState<Metro[]>([]);
    const [bus, setBus] = useState<Transportation[]>([]);

    useEffect(() => {
        const fetchTransports = async () => {
            const accessToken = await getToken();
            if (!accessToken) return;

            const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");

            try {
                const res = await fetch(API_ROUTES.trans.timeByProfile(profile.id), {
                    headers: {
                        Authorization: `Bearer ${accessToken}`
                    }
                });

                if (res.ok) {
                    const data = await res.json();
                    const all: Transportation[] = data;
                    // BUS 데이터 분류
                    const busList = all.filter((item) => item.type === "BUS");
                    setBus(busList);

                    // METRO 데이터 가공
                    const metroGrouped: { [key: string]: Metro } = {};

                    all.forEach((item) => {
                        if (item.type === "METRO") {
                            const key = `${item.number}-${item.station}`;

                            const directionOnly = item.direction.split("-")[1]?.trim() || "";

                            if (!metroGrouped[key]) {
                                metroGrouped[key] = {
                                    id: item.id,
                                    number: item.number,
                                    station: item.station,
                                    information: {
                                        dir1: directionOnly,
                                        info1: item.information,
                                        dir2: "",
                                        info2: ""
                                    }
                                };
                            } else {
                                metroGrouped[key].information.dir2 = directionOnly;
                                metroGrouped[key].information.info2 = item.information;
                            }
                        }
                    });

                    const metroList = Object.values(metroGrouped);
                    setMetros(metroList);
                } else {
                    console.error("교통 정보 불러오기 실패");
                }
            } catch (err) {
                console.error("네트워크 오류:", err);
            }
        };
        fetchTransports();
    }, []);

    return (
        <div className="py-1 px-5 h-auto w-52">
            {/* 🚍 버스 정보 */}
            {bus.map((item, index) => (
                <div
                    className={`py-2 ${index !== bus.length - 1 ? "border-b border-white" : ""} ${
                        index == bus.length - 1 && metros.length > 0 ? "border-b border-white" : ""
                    }`}
                >
                    <div key={`bus-${index}`} className="flex items-center">
                        <div className="text-2xl mr-3">
                            <FaBus />
                        </div>
                        <div className="text-left flex-grow">
                            <p className="text-sm font-bold">
                                {item.station.length < 8 ? item.station : item.station.slice(0, 8) + "..."}
                            </p>
                            <p>{item.number}</p>
                        </div>
                    </div>
                    <p className="text-xs font-bold py-1">
                        {item.direction && item.direction.length > 8
                            ? item.direction.slice(0, 8) + ".."
                            : item.direction}{" "}
                        방면
                    </p>
                    <p className="text-xs">{item.information}</p>
                </div>
            ))}

            {/* 🚇 지하철 정보 */}
            {metros.map((item, index) => (
                <div className={`py-2 ${index !== metros.length - 1 ? "border-b border-white" : ""} `}>
                    <div key={`metros-${index}`} className="flex items-center py-2">
                        <div className="text-2xl mr-3">
                            <FaSubway />
                        </div>
                        <div className="text-left  flex gap-2 items-center">
                            <p className="text-sm font-bold">
                                {item.station.length < 8 ? item.station : item.station.slice(0, 8) + "..."}
                            </p>

                            <p>{item.number}</p>
                        </div>
                    </div>
                    <div className="flex flex-col gap-1">
                        <div className="flex flex-col gap-1">
                            <p className="text-xs font-bold">{item.information.dir1}</p>
                            <p className="text-xs">{item.information.info1}</p>
                        </div>
                        <div className="flex flex-col gap-1">
                            <p className="text-xs font-bold">{item.information.dir2}</p>
                            <p className="text-xs">{item.information.info2}</p>
                        </div>
                    </div>
                </div>
            ))}
        </div>
    );
}
