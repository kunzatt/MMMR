"use client";

import { useEffect } from "react";
import { ImCross } from "react-icons/im";

interface PostcodeData {
    zonecode: string;
    roadAddress: string;
    jibunAddress: string;
    userSelectedType: "R" | "J";
    buildingName: string;
    apartment: "Y" | "N";
    bname: string;
}

interface SearchAdProps {
    onComplete: (address: string, zonecode: string) => void;
    onClose: () => void;
}

declare global {
    interface Window {
        daum: {
            Postcode: new (options: {
                oncomplete: (data: PostcodeData) => void;
                width?: string;
                height?: string;
                maxSuggestItems?: number;
            }) => {
                embed: (element: HTMLElement) => void;
            };
        };
    }
}

export {};

let scriptLoaded = false; // ✅ 전역 변수로 스크립트 중복 로딩 방지

export default function SearchAd({ onComplete, onClose }: SearchAdProps) {
    useEffect(() => {
        const elementLayer = document.getElementById("daum-postcode");
        if (!elementLayer) return;

        // 기존 iframe 제거
        elementLayer.innerHTML = "";

        // 스크립트가 아직 로드되지 않은 경우에만 추가
        if (!scriptLoaded) {
            const script = document.createElement("script");
            script.src = "//t1.daumcdn.net/mapjsapi/bundle/postcode/prod/postcode.v2.js";
            script.onload = () => {
                scriptLoaded = true;
                openPostcode();
            };
            document.body.appendChild(script);
        } else {
            openPostcode();
        }

        function openPostcode() {
            new window.daum.Postcode({
                oncomplete: function (data: PostcodeData) {
                    let addr = "";
                    let extraAddr = "";

                    if (data.userSelectedType === "R") {
                        addr = data.roadAddress;
                    } else {
                        addr = data.jibunAddress;
                    }

                    if (data.userSelectedType === "R") {
                        if (data.bname !== "" && /[\uac00-\ud7af]+[동|로|가]$/g.test(data.bname)) {
                            extraAddr += data.bname;
                        }
                        if (data.buildingName !== "" && data.apartment === "Y") {
                            extraAddr += extraAddr !== "" ? ", " + data.buildingName : data.buildingName;
                        }
                        if (extraAddr !== "") {
                            addr += ` (${extraAddr})`;
                        }
                    }

                    onComplete(addr, data.zonecode);
                    onClose();
                },
                width: "100%",
                height: "100%",
                maxSuggestItems: 5,
            }).embed(elementLayer as HTMLElement);
        }

        return () => {
            if (elementLayer) elementLayer.innerHTML = "";
        };
    }, [onComplete, onClose]);

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-md h-[500px] relative p-2 rounded-lg overflow-hidden">
                <div id="daum-postcode" style={{ width: "100%", height: "100%", position: "relative" }}></div>
                <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                    <ImCross />
                </button>
            </div>
        </div>
    );
}
