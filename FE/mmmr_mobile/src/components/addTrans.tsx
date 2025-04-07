"use client";
import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { ImCross } from "react-icons/im";
import { FiSearch } from "react-icons/fi";
import API_ROUTES from "@/config/apiRoutes";

interface AddTransProps {
    onClose: () => void;
}

interface Trans {
    type: "METRO";
    station: string;
    sequence: number;
    number: string; // METRO일 때는 string
    information: string;
}

export default function AddTrans({ onClose }: AddTransProps) {
    const router = useRouter();

    const [type, setType] = useState<"BUS" | "METRO">("BUS");

    const [searchResults, setSearchResults] = useState<Trans[]>([]);
    const [selectedStation, setSelectedStation] = useState<Trans>(); // 선택된 정류장

    const [subwayLines, setSubwayLines] = useState<Trans[]>([]);
    const [busNumbers, setBusNumbers] = useState<Trans[]>([]);
    const [selectedNumber, setSelectedNumber] = useState<Trans | null>();

    const [searchType, setSearchType] = useState("number"); // 'number' or 'station'
    const [searchKeyword, setSearchKeyword] = useState("");

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/login");
            return null;
        }

        try {
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`
                },
                body: JSON.stringify({ token: accessToken })
            });

            if (!validateResponse.ok && refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    },
                    body: JSON.stringify({ token: refreshToken })
                });

                const refreshData = await refreshResponse.json();
                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    localStorage.setItem("accessToken", accessToken!);
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/login");
                    return null;
                }
            }

            return accessToken;
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const handleSearch = async () => {
        setSearchResults([]);
        setBusNumbers([]);
        setSubwayLines([]);
        const accessToken = await getTokens();
        if (type == "BUS") {
            if (searchType == "number") {
                try {
                    const response = await fetch(
                        `${API_ROUTES.trans.search}?type=${type}&keyword=${encodeURIComponent(searchKeyword)}`,
                        {
                            method: "GET",
                            headers: {
                                "Content-Type": "application/json",
                                "Authorization": `Bearer ${accessToken}`
                            }
                        }
                    );
                    if (response.ok) {
                        const data = await response.json();
                        const filtered = data.data.filter((item: Trans) => item.number === searchKeyword);
                        setSearchResults(filtered.map((item: Trans) => item));
                    } else {
                        const errorData = await response.json();
                        console.error("추가 실패:", errorData);
                        alert("추가에 실패했습니다.");
                    }
                } catch (error) {
                    console.error("추가 중 오류:", error);
                    alert("네트워크 오류가 발생했습니다.");
                }
            }
            if (searchType == "station") {
                try {
                    const response = await fetch(
                        `${API_ROUTES.trans.search}?type=${type}&keyword=${encodeURIComponent(searchKeyword)}`,
                        {
                            method: "GET",
                            headers: {
                                "Content-Type": "application/json",
                                "Authorization": `Bearer ${accessToken}`
                            }
                        }
                    );
                    if (response.ok) {
                        const data = await response.json();
                        const filtered = data.data.filter((item: Trans) => item.station.includes(searchKeyword));
                        const seen = new Set();
                        const uniqueStations = filtered.filter((item: Trans) => {
                            const normalized = item.station.normalize("NFKC").replace(/\s+/g, "").replace(/[․.]/g, "");
                            if (seen.has(normalized)) return false;
                            seen.add(normalized);
                            return true;
                        });

                        setSearchResults(uniqueStations);
                    } else {
                        const errorData = await response.json();
                        console.error("추가 실패:", errorData);
                        alert("추가에 실패했습니다.");
                    }
                } catch (error) {
                    console.error("추가 중 오류:", error);
                    alert("네트워크 오류가 발생했습니다.");
                }
            }
        } else {
            try {
                const response = await fetch(
                    `${API_ROUTES.trans.search}?type=${type}&keyword=${encodeURIComponent(searchKeyword)}`,
                    {
                        method: "GET",
                        headers: {
                            "Content-Type": "application/json",
                            "Authorization": `Bearer ${accessToken}`
                        }
                    }
                );
                if (response.ok) {
                    const data = await response.json();
                    const filtered = data.data.filter((item: Trans) => item.station.includes(searchKeyword));
                    const seen = new Set();
                    const uniqueStations = filtered.filter((item: Trans) => {
                        const normalizedStation = item.station.normalize("NFKC").replace(/\s+/g, "");
                        if (seen.has(normalizedStation)) return false;
                        seen.add(normalizedStation);
                        return true;
                    });

                    setSearchResults(uniqueStations);
                } else {
                    const errorData = await response.json();
                    console.error("검색 실패:", errorData);
                    alert("검색에 실패했습니다.");
                }
            } catch (error) {
                console.error("검색 중 오류:", error);
                alert("네트워크 오류가 발생했습니다.");
            }
        }
    };

    const fetchBusNumbersAtStation = async (stationName: string) => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(
                `${API_ROUTES.trans.search}?type=BUS&keyword=${encodeURIComponent(stationName)}`,
                {
                    method: "GET",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    }
                }
            );

            if (response.ok) {
                const data = await response.json();
                setBusNumbers(data.data);
            } else {
                console.error("버스 번호 조회 실패");
            }
        } catch (error) {
            console.error("버스 번호 불러오기 오류:", error);
        }
    };

    const fetchMetroLine = async (stationName: string) => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(
                `${API_ROUTES.trans.search}?type=METRO&keyword=${encodeURIComponent(stationName.slice(0, -1))}`,
                {
                    method: "GET",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`
                    }
                }
            );

            if (response.ok) {
                const data = await response.json();
                const filtered = data.data.filter((item: Trans) => item.station === stationName);
                setSubwayLines(filtered);
            } else {
                console.error("지하철 호선 조회 실패");
            }
        } catch (error) {
            console.error("지하철 호선 불러오기 오류:", error);
        }
    };

    const parseInformation = (infoString: string) => {
        const pairs = infoString.split(",").map((s) => s.trim());
        const result: { [key: string]: number } = {};

        pairs.forEach((pair) => {
            const [key, value] = pair.split(":").map((s) => s.trim());
            result[key] = Number(value);
        });

        return result;
    };

    const handleAdd = async () => {
        if (type == "BUS") {
            if (!selectedStation) {
                alert("정류장을 선택해주세요.");
                return;
            }

            const accessToken = await getTokens();
            if (!accessToken) return;

            const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
            const profileId = profile?.id;
            if (!profileId) {
                alert("프로필 정보가 없습니다.");
                return;
            }

            const parsed = parseInformation(selectedStation.information);

            const payload = {
                profileId,
                type: selectedStation.type,
                number: selectedStation.number.toString(),
                station: selectedStation.station,
                routeId: parsed.routeId,
                stationId: parsed.stationId
            };
            try {
                const response = await fetch(API_ROUTES.trans.add, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${accessToken}`
                    },
                    body: JSON.stringify(payload)
                });

                if (response.ok) {
                    alert("교통 정보가 추가되었습니다.");
                    onClose();
                } else {
                    const error = await response.json();
                    console.error("추가 실패:", error);
                    alert("추가에 실패했습니다.");
                }
            } catch (error) {
                console.error("추가 중 오류:", error);
                alert("네트워크 오류가 발생했습니다.");
            }
        } else {
            if (!selectedNumber) {
                alert("역을 선택해주세요.");
                return;
            }
            const accessToken = await getTokens();
            if (!accessToken) return;

            const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
            const profileId = profile?.id;
            if (!profileId) {
                alert("프로필 정보가 없습니다.");
                return;
            }

            const payload = {
                profileId,
                type: selectedNumber.type,
                number: selectedNumber.number,
                station: selectedNumber.station.slice(0, -1)
            };
            try {
                const response = await fetch(API_ROUTES.trans.add, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${accessToken}`
                    },
                    body: JSON.stringify(payload)
                });

                if (response.ok) {
                    alert("교통 정보가 추가되었습니다.");
                    onClose();
                } else {
                    const error = await response.json();
                    console.error("추가 실패:", error);
                    alert("추가에 실패했습니다.");
                }
            } catch (error) {
                console.error("추가 중 오류:", error);
                alert("네트워크 오류가 발생했습니다.");
            }
        }
    };

    useEffect(() => {
        if (selectedStation) {
            if (type == "METRO") fetchMetroLine(selectedStation.station);
            if (type == "BUS" && searchType == "station") fetchBusNumbersAtStation(selectedStation.station);
        }
    }, [selectedStation]);

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-sm rounded-lg p-6 relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-4">교통정보 추가</h2>
                <div className="space-y-3">
                    <div className="flex justify-center mb-4 space-x-2">
                        <button
                            onClick={() => {
                                setType("BUS");
                                setSearchResults([]);
                                setSearchKeyword("");
                            }}
                            className={`px-4 py-1 rounded-full border ${
                                type === "BUS" ? "bg-blue-200 font-semibold" : "bg-white"
                            }`}
                        >
                            버스
                        </button>
                        <button
                            onClick={() => {
                                setType("METRO");
                                setSearchResults([]);
                                setBusNumbers([]);
                                setSearchKeyword("");
                            }}
                            className={`px-4 py-1 rounded-full border ${
                                type === "METRO" ? "bg-blue-200 font-semibold" : "bg-white"
                            }`}
                        >
                            지하철
                        </button>
                    </div>

                    {/* 결과 리스트 or 노선 드롭다운 */}
                    {type === "BUS" ? (
                        <div>
                            {/* 검색창 */}
                            <div className="mb-4">
                                <div className="flex items-center border rounded-md px-2">
                                    {/* 검색 기준 선택 */}
                                    <select
                                        value={searchType}
                                        onChange={(e) => {
                                            setSearchType(e.target.value);
                                            setSearchResults([]);
                                            setBusNumbers([]);
                                            setSearchKeyword("");
                                        }}
                                        className="p-1 bg-white text-sm text-gray-600 border-r mr-2 focus:outline-none"
                                    >
                                        <option value="number">번호</option>
                                        <option value="station">정류장</option>
                                    </select>

                                    {/* 검색어 입력 */}
                                    <input
                                        type="text"
                                        value={searchKeyword}
                                        onChange={(e) => setSearchKeyword(e.target.value)}
                                        className="flex-1 min-w-0 p-2 focus:outline-none"
                                        placeholder={searchType === "number" ? "버스 번호 입력" : "정류장 이름 입력"}
                                    />

                                    {/* 검색 버튼 */}
                                    <button onClick={handleSearch} className="ml-2 text-gray-600 hover:text-black">
                                        <FiSearch size={20} />
                                    </button>
                                </div>
                            </div>
                            {searchResults.length > 0 && (
                                <div>
                                    <label className="block text-sm text-gray-500 mb-1">정류장 선택</label>
                                    <div className="border rounded-md p-2 h-24 overflow-y-auto text-sm space-y-1">
                                        {searchResults.map((result, i) => (
                                            <div
                                                key={i}
                                                onClick={() => setSelectedStation(result)}
                                                className={`cursor-pointer px-2 py-1 rounded-md ${
                                                    selectedStation === result
                                                        ? "bg-blue-100 font-semibold text-blue-700"
                                                        : "hover:bg-gray-100"
                                                }`}
                                            >
                                                {result.station}
                                            </div>
                                        ))}
                                    </div>
                                </div>
                            )}
                            {searchResults.length > 0 && busNumbers.length > 0 && (
                                <div className="mt-3">
                                    <label className="block text-sm text-gray-500 mb-1">버스 선택</label>
                                    <div className="border rounded-md p-2 h-24 overflow-y-auto text-sm space-y-1">
                                        {busNumbers.map((num, idx) => (
                                            <div
                                                key={idx}
                                                onClick={() => setSelectedNumber(num)}
                                                className={`cursor-pointer px-2 py-1 rounded-md ${
                                                    selectedNumber === num
                                                        ? "bg-blue-100 font-semibold text-blue-700"
                                                        : "hover:bg-gray-100"
                                                }`}
                                            >
                                                {num.number}
                                            </div>
                                        ))}
                                    </div>
                                </div>
                            )}
                        </div>
                    ) : (
                        <div>
                            <div className="mb-4">
                                <div className="flex items-center border rounded-md px-2">
                                    {/* 검색어 입력 */}
                                    <input
                                        type="text"
                                        value={searchKeyword}
                                        onChange={(e) => setSearchKeyword(e.target.value)}
                                        className="flex-1 p-2 focus:outline-none"
                                        placeholder={"역 이름 입력"}
                                    />

                                    {/* 검색 버튼 */}
                                    <button onClick={handleSearch} className="ml-2 text-gray-600 hover:text-black">
                                        <FiSearch size={20} />
                                    </button>
                                </div>
                            </div>
                            {searchResults.length > 0 && (
                                <div>
                                    <label className="block text-sm text-gray-500 mb-1">역 선택</label>
                                    <div className="border rounded-md p-2 h-24 overflow-y-auto text-sm space-y-1">
                                        {searchResults.map((result, i) => (
                                            <div
                                                key={i}
                                                onClick={() => setSelectedStation(result)}
                                                className={`cursor-pointer px-2 py-1 rounded-md ${
                                                    selectedStation === result
                                                        ? "bg-blue-100 font-semibold text-blue-700"
                                                        : "hover:bg-gray-100"
                                                }`}
                                            >
                                                {result.station}
                                            </div>
                                        ))}
                                    </div>
                                </div>
                            )}
                            {searchResults.length > 0 && subwayLines.length > 0 && (
                                <div className="mt-3">
                                    <label className="block text-sm text-gray-500 mb-1">호선 선택</label>
                                    <div className="border rounded-md p-2 h-24 overflow-y-auto text-sm space-y-1">
                                        {subwayLines.map((num, idx) => (
                                            <div
                                                key={idx}
                                                onClick={() => setSelectedNumber(num)}
                                                className={`cursor-pointer px-2 py-1 rounded-md ${
                                                    selectedNumber === num
                                                        ? "bg-blue-100 font-semibold text-blue-700"
                                                        : "hover:bg-gray-100"
                                                }`}
                                            >
                                                {num.number}
                                            </div>
                                        ))}
                                    </div>
                                </div>
                            )}
                        </div>
                    )}
                </div>
                <div className="flex justify-center mt-6">
                    <button className="bg-blue-300 text-white py-2 px-4 rounded-md w-1/3" onClick={handleAdd}>
                        추가
                    </button>
                    <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                        <ImCross />
                    </button>
                </div>
            </div>
        </div>
    );
}
