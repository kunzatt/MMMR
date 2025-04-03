"use client";
import API_ROUTES from "@/config/apiRoutes";
import { getToken } from "@/config/getToken";
import dayjs from "dayjs";
import isSameOrAfter from "dayjs/plugin/isSameOrAfter";
import { useEffect, useState } from "react";
dayjs.extend(isSameOrAfter);

interface Schedule {
    id: number;
    title: string;
    profileId: number;
    startDate: string; // ISO 날짜 문자열
    endDate: string;
}

export default function Schedule() {
    const [schedules, setSchedules] = useState<Schedule[]>([]);

    useEffect(() => {
        const fetchSchedules = async () => {
            const accessToken = await getToken();
            const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
            const profileId = profile?.id;

            try {
                const response = await fetch(`${API_ROUTES.schedule.listByProfile}?profileId=${profileId}`, {
                    method: "GET",
                    headers: {
                        Authorization: `Bearer ${accessToken}`
                    }
                });

                if (response.ok) {
                    const data = await response.json();
                    setSchedules(data.data || []);
                } else {
                    console.error("일정 조회 실패");
                }
            } catch (error) {
                console.error("일정 요청 오류:", error);
            }
        };
        fetchSchedules();
    }, []);

    const sortedGroupByDate = (items: Schedule[]) => {
        const today = dayjs().startOf("day"); // 오늘 날짜 00:00

        const filtered = items
            .filter((item) => dayjs(item.startDate).isSameOrAfter(today))
            .sort((a, b) => dayjs(a.startDate).unix() - dayjs(b.startDate).unix())
            .slice(0, 4); // 상위 4개만 자르기

        const grouped = filtered.reduce((acc: Record<string, Schedule[]>, item) => {
            const dateKey = dayjs(item.startDate).format("MMM D");
            acc[dateKey] = acc[dateKey] || [];
            acc[dateKey].push(item);
            return acc;
        }, {});

        return Object.entries(grouped)
            .sort((a, b) => dayjs(a[1][0].startDate).unix() - dayjs(b[1][0].startDate).unix())
            .reduce((acc: Record<string, Schedule[]>, [key, val]) => {
                acc[key] = val;
                return acc;
            }, {});
    };

    const groupedSchedules = sortedGroupByDate(schedules);

    return (
        <div className="py-3 px-5 h-auto w-52">
            {Object.entries(groupedSchedules).length > 0 ? (
                Object.entries(groupedSchedules).map(([date, items], index) => (
                    <div
                        key={index}
                        className={`${index === Object.entries(groupedSchedules).length - 1 ? "" : "mb-2"}`}
                    >
                        <h3 className="text-lg font-semibold border-b-2 border-white">{date}</h3>
                        <div className="pt-2 flex flex-col gap-2">
                            {items.map((schedule) => (
                                <div
                                    key={schedule.id}
                                    className="px-3 py-1 border border-white rounded-md text-sm break-keep text-center"
                                >
                                    {schedule.title}
                                </div>
                            ))}
                        </div>
                    </div>
                ))
            ) : (
                <p>일정이 없습니다</p>
            )}
        </div>
    );
}
